// $Id$

/*
 Copyright (c) 2007-2015, Trustees of The Leland Stanford Junior University
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.
 Redistributions in binary form must reproduce the above copyright notice, this
 list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*mesh.cpp
 *
 * 2D Mesh with Fat Bus Cross node annotation.
 *
 * Node layout for k=5, n=2 (row-major, dim=0 is X/col, dim=1 is Y/row):
 *
 *   col:  0   1  [2]  3   4
 *   row 0: 0   1   2   3   4
 *   row 1: 5   6   7   8   9
 *  [row 2]:10  11 [12] 13  14   ← center row
 *   row 3: 15  16  17  18  19
 *   row 4: 20  21  22  23  24
 *
 *   is_cross_node  = {2,7,10,11,12,13,14,17,22}  (row==2 OR col==2)
 *   is_center_node = {12}                          (row==2 AND col==2)
 */

#include <cassert>
#include <sstream>
#include <vector>
#include "booksim.hpp"
#include "mesh.hpp"
#include "buffer.hpp"
#include "buffer_state.hpp"
#include "iq_router.hpp"

// Definition of the global Mesh pointer declared in mesh.hpp.
Mesh *gMeshNet = nullptr;

Mesh::Mesh( const Configuration &config, const string &name )
  : KNCube( config, name, /*mesh=*/true, /*defer_build=*/true )
{
  assert( GetN() == 2 && "Mesh Fat Bus Cross marking requires n=2 (2D mesh)" );

  // Cross/center marks must be set before computing channel counts.
  _MarkCrossNodes();

  // Set global pointer only after _is_cross_node / _is_center_node are fully
  // populated, so any code that dereferences gMeshNet always sees a consistent
  // node-type state.
  gMeshNet = this;

  const int k = GetK();
  const int n = GetN();

  // Helper: compute k^dim for integer dim.
  auto kpow = [&]( int dim ) {
    int r = 1;
    for ( int d = 0; d < dim; ++d ) r *= k;
    return r;
  };

  // Helper: right neighbor of node in dimension dim (wraps at boundary).
  auto right_node = [&]( int node, int dim ) {
    int kd  = kpow( dim );
    int loc = ( node / kd ) % k;
    return ( loc == k - 1 ) ? node - ( k - 1 ) * kd : node + kd;
  };

  // Helper: left neighbor of node in dimension dim (wraps at boundary).
  auto left_node = [&]( int node, int dim ) {
    int kd  = kpow( dim );
    int loc = ( node / kd ) % k;
    return ( loc == 0 ) ? node + ( k - 1 ) * kd : node - kd;
  };

  // Count total directed channels needed for the fat-bus topology.
  // Each (node, dim, dir) slot contributes _NumParallel(node, neighbor) channels.
  int total_chans = 0;
  for ( int node = 0; node < _size; ++node ) {
    for ( int dim = 0; dim < n; ++dim ) {
      total_chans += _NumParallel( node, right_node( node, dim ) );
      total_chans += _NumParallel( node, left_node( node, dim ) );
    }
  }

  // KNCube's _ComputeSize already allocated 2*n*_size base channels via _Alloc().
  // Allocate the additional parallel channels beyond the base count.
  int base_chans = _channels;  // = 2 * n * _size set by KNCube::_ComputeSize
  for ( int c = base_chans; c < total_chans; ++c ) {
    ostringstream fname;
    fname << Name() << "_fchan_" << c;
    FlitChannel *fc = new FlitChannel( this, fname.str(), _classes );
    _timed_modules.push_back( fc );
    _chan.push_back( fc );

    ostringstream cname;
    cname << Name() << "_cchan_" << c;
    CreditChannel *cc = new CreditChannel( this, cname.str() );
    _timed_modules.push_back( cc );
    _chan_cred.push_back( cc );
  }
  _channels = total_chans;

  // Build the network now that all channels are allocated and cross marks are set.
  _BuildNet( config );
}

Mesh::~Mesh()
{
  for ( int node = 0; node < _size; ++node ) {
    for ( auto *ch : _inject_aux[node]      ) delete ch;
    for ( auto *ch : _inject_aux_cred[node] ) delete ch;
    for ( auto *ch : _eject_extra[node]      ) delete ch;
    for ( auto *ch : _eject_extra_cred[node] ) delete ch;
  }
}

void Mesh::RegisterRoutingFunctions()
{
  KNCube::RegisterRoutingFunctions();
}

void Mesh::_MarkCrossNodes()
{
  _is_cross_node.assign( _nodes, false );
  _is_center_node.assign( _nodes, false );

  // Integer center index in each dimension (e.g. k=5 → center=2).
  int center = GetK() / 2;

  for ( int node = 0; node < _nodes; ++node ) {
    // dim=0 (X): position changes by 1 per step → col = node % k
    // dim=1 (Y): position changes by k per step → row = (node / k) % k
    int col = node % GetK();
    int row = ( node / GetK() ) % GetK();

    bool on_center_col = ( col == center );
    bool on_center_row = ( row == center );

    _is_cross_node[node]  = on_center_col || on_center_row;
    _is_center_node[node] = on_center_col && on_center_row;
  }
}

int Mesh::_NumParallel( int a, int b ) const
{
  if ( IsCenterNode( a ) || IsCenterNode( b ) ) return 4;
  if ( IsCrossNode( a )  || IsCrossNode( b )  ) return 2;
  return 1;
}

int Mesh::_NumInjectionSlots( int node ) const
{
  if ( IsCenterNode( node ) ) return 4;  // N / S / E / W
  if ( IsCrossNode( node )  ) return 2;  // Phase4: directional (west/east or north/south)
  return 2;                               // Px (XY) / Py (YX)
}

int Mesh::_NumEjectionSlots( int node ) const
{
  if ( IsCenterNode( node ) ) return 4;  // N / S / E / W
  return 2;                               // outer: Px/Py, cross: 2 directional
}

void Mesh::_BuildNet( const Configuration &config )
{
  const int k = GetK();
  const int n = GetN();
  const bool use_noc_latency = ( config.GetInt( "use_noc_latency" ) == 1 );

  // Helper: k^dim.
  auto kpow = [&]( int dim ) {
    int r = 1;
    for ( int d = 0; d < dim; ++d ) r *= k;
    return r;
  };

  // Neighbors (same wrap-around logic as KNCube::_LeftNode / _RightNode).
  auto right_nb = [&]( int node, int dim ) {
    int kd  = kpow( dim );
    int loc = ( node / kd ) % k;
    return ( loc == k - 1 ) ? node - ( k - 1 ) * kd : node + kd;
  };
  auto left_nb = [&]( int node, int dim ) {
    int kd  = kpow( dim );
    int loc = ( node / kd ) % k;
    return ( loc == 0 ) ? node + ( k - 1 ) * kd : node - kd;
  };

  // ----------------------------------------------------------------
  // Pass 1: Compute per-slot channel start indices and parallel counts.
  //
  // Slot encoding: node * n * 2 + dim * 2 + dir
  //   dir = 0 → right (toward increasing dim coordinate)
  //   dir = 1 → left  (toward decreasing dim coordinate)
  // ----------------------------------------------------------------
  const int slots = _size * n * 2;
  vector<int> par_cnt( slots );
  vector<int> chan_start( slots );

  int chan_idx = 0;
  for ( int node = 0; node < _size; ++node ) {
    for ( int dim = 0; dim < n; ++dim ) {
      // Right slot (dir = 0): channels FROM node TO right_nb(node, dim)
      int sr = node * n * 2 + dim * 2 + 0;
      par_cnt[sr]    = _NumParallel( node, right_nb( node, dim ) );
      chan_start[sr] = chan_idx;
      chan_idx       += par_cnt[sr];

      // Left slot (dir = 1): channels FROM node TO left_nb(node, dim)
      int sl = node * n * 2 + dim * 2 + 1;
      par_cnt[sl]    = _NumParallel( node, left_nb( node, dim ) );
      chan_start[sl] = chan_idx;
      chan_idx       += par_cnt[sl];
    }
  }
  assert( chan_idx == _channels );

  // ----------------------------------------------------------------
  // Allocate auxiliary injection NI channels (slots 1+) per node.
  // Slot 0 is the base-class _inject[node]; extra slots live here.
  // ----------------------------------------------------------------
  _inject_aux.resize( _size );
  _inject_aux_cred.resize( _size );
  for ( int node = 0; node < _size; ++node ) {
    int ni = _NumInjectionSlots( node );
    _inject_aux[node].resize( ni - 1, nullptr );
    _inject_aux_cred[node].resize( ni - 1, nullptr );
    for ( int s = 1; s < ni; ++s ) {
      ostringstream fname, cname;
      fname << Name() << "_fchan_ingress" << node << "_" << s;
      cname << Name() << "_cchan_ingress" << node << "_" << s;
      auto *fc = new FlitChannel( this, fname.str(), _classes );
      auto *cc = new CreditChannel( this, cname.str() );
      fc->SetSource( nullptr, node );
      _timed_modules.push_back( fc );
      _timed_modules.push_back( cc );
      _inject_aux[node][s - 1]      = fc;
      _inject_aux_cred[node][s - 1] = cc;
    }
  }

  // ----------------------------------------------------------------
  // Allocate auxiliary ejection output channels (slots 1+) per node.
  // Slot 0 is the base-class _eject[node]; extra slots live here.
  // Only center node currently has ne > 1.
  // ----------------------------------------------------------------
  _eject_extra.resize( _size );
  _eject_extra_cred.resize( _size );
  for ( int node = 0; node < _size; ++node ) {
    int ne = _NumEjectionSlots( node );
    _eject_extra[node].resize( ne - 1, nullptr );
    _eject_extra_cred[node].resize( ne - 1, nullptr );
    for ( int s = 1; s < ne; ++s ) {
      ostringstream fname, cname;
      fname << Name() << "_fchan_egress" << node << "_" << s;
      cname << Name() << "_cchan_egress" << node << "_" << s;
      auto *fc = new FlitChannel( this, fname.str(), _classes );
      auto *cc = new CreditChannel( this, cname.str() );
      _timed_modules.push_back( fc );
      _timed_modules.push_back( cc );
      _eject_extra[node][s - 1]      = fc;
      _eject_extra_cred[node][s - 1] = cc;
    }
  }

  // ----------------------------------------------------------------
  // Pass 2: Create routers with the correct per-node port counts.
  //
  // out_ports = routing channels + 1 ejection  (unchanged)
  // in_ports  = routing channels + ni_count    (extra NI input ports)
  // ----------------------------------------------------------------
  ostringstream rname;
  for ( int node = 0; node < _size; ++node ) {
    // Routing channel count (same for inputs and outputs).
    int routing = 0;
    for ( int dim = 0; dim < n; ++dim ) {
      routing += par_cnt[ node * n * 2 + dim * 2 + 0 ];  // right
      routing += par_cnt[ node * n * 2 + dim * 2 + 1 ];  // left
    }

    int ni_count = _NumInjectionSlots( node );
    int ne_count = _NumEjectionSlots( node );
    int in_ports  = routing + ni_count;  // extra NI inject slots
    int out_ports = routing + ne_count;  // ejection port(s): 1 normally, 4 for center

    rname.str( "" );
    rname << "router";
    if ( k > 1 ) {
      for ( int dim_off = _size / k; dim_off >= 1; dim_off /= k ) {
        rname << "_" << ( node / dim_off ) % k;
      }
    }

    _routers[node] = Router::NewRouter( config, this, rname.str(),
                                        node, in_ports, out_ports );
    _timed_modules.push_back( _routers[node] );
  }

  // ----------------------------------------------------------------
  // Pass 3: Connect channels to routers, following BookSim convention:
  //   AddInputChannel  on the sink   router (flit arrives here)
  //   AddOutputChannel on the source router (flit departs here)
  //
  // For directed link A → B the channel object lives in slot A's right/left
  // slot.  Both the source (A) and the sink (B) register the same channel
  // object: source via AddOutputChannel, sink via AddInputChannel.
  // ----------------------------------------------------------------
  for ( int node = 0; node < _size; ++node ) {
    for ( int dim = 0; dim < n; ++dim ) {
      const int rn = right_nb( node, dim );
      const int ln = left_nb(  node, dim );

      // ------ Input channels arriving at node ------
      // From rn: rn sends leftward → rn's left slot (dir=1)
      {
        int slot = rn * n * 2 + dim * 2 + 1;
        for ( int ch = 0; ch < par_cnt[slot]; ++ch ) {
          int c = chan_start[slot] + ch;
          _routers[node]->AddInputChannel( _chan[c], _chan_cred[c] );
          _chan[c]->SetLatency( 1 );
          _chan_cred[c]->SetLatency( 1 );
        }
      }
      // From ln: ln sends rightward → ln's right slot (dir=0)
      {
        int slot = ln * n * 2 + dim * 2 + 0;
        for ( int ch = 0; ch < par_cnt[slot]; ++ch ) {
          int c = chan_start[slot] + ch;
          _routers[node]->AddInputChannel( _chan[c], _chan_cred[c] );
          _chan[c]->SetLatency( 1 );
          _chan_cred[c]->SetLatency( 1 );
        }
      }

      // ------ Output channels leaving node ------
      // To rn: node's right slot (dir=0)
      {
        int slot = node * n * 2 + dim * 2 + 0;
        for ( int ch = 0; ch < par_cnt[slot]; ++ch ) {
          int c = chan_start[slot] + ch;
          _routers[node]->AddOutputChannel( _chan[c], _chan_cred[c] );
          _chan[c]->SetLatency( 1 );
          _chan_cred[c]->SetLatency( 1 );
        }
      }
      // To ln: node's left slot (dir=1)
      {
        int slot = node * n * 2 + dim * 2 + 1;
        for ( int ch = 0; ch < par_cnt[slot]; ++ch ) {
          int c = chan_start[slot] + ch;
          _routers[node]->AddOutputChannel( _chan[c], _chan_cred[c] );
          _chan[c]->SetLatency( 1 );
          _chan_cred[c]->SetLatency( 1 );
        }
      }
    }

    // Injection NI slot 0 (primary, managed by base class).
    _routers[node]->AddInputChannel( _inject[node], _inject_cred[node] );
    _inject[node]->SetLatency( 1 );

    // Injection NI slots 1+ (auxiliary, node-type dependent).
    for ( int s = 0; s < (int)_inject_aux[node].size(); ++s ) {
      _routers[node]->AddInputChannel( _inject_aux[node][s],
                                       _inject_aux_cred[node][s] );
      _inject_aux[node][s]->SetLatency( 1 );
      _inject_aux_cred[node][s]->SetLatency( 1 );
    }

    // Ejection slot 0 (primary, base-class channel).
    _routers[node]->AddOutputChannel( _eject[node], _eject_cred[node] );
    _eject[node]->SetLatency( 1 );

    // Ejection slots 1+ (center node only: S, E, W).
    for ( int s = 0; s < (int)_eject_extra[node].size(); ++s ) {
      _routers[node]->AddOutputChannel( _eject_extra[node][s],
                                        _eject_extra_cred[node][s] );
      _eject_extra[node][s]->SetLatency( 1 );
      _eject_extra_cred[node][s]->SetLatency( 1 );
    }
  }

  // ----------------------------------------------------------------
  // Pass 4: Per-link VC buffer size adjustment.
  //
  // Both sides of each link must be updated consistently:
  //   Buffer::_size          (input buffer at receiving router)
  //   BufferState::_vc_buf_size (upstream router's downstream tracking)
  //
  // The multiplier equals _NumParallel(a,b): 1 / 2 / 4 for
  // outer / cross / center links — the same scaling used for
  // parallel channels.
  //
  // Input port order per node (matches Pass 3 AddInputChannel order):
  //   for each dim: [from rn (par_E/par_S)] [from ln (par_W/par_N)]
  //   then: injection slots  ← skipped (NI ports, no link multiplier)
  //
  // Output port order per node (matches Pass 3 AddOutputChannel order):
  //   for each dim: [to rn (par_E/par_S)] [to ln (par_W/par_N)]
  //   then: ejection slot    ← skipped
  // ----------------------------------------------------------------
  const int base_vc_buf = config.GetInt( "vc_buf_size" );
  const int num_vcs     = config.GetInt( "num_vcs" );

  for ( int node = 0; node < _size; ++node ) {
    IQRouter * rtr = static_cast<IQRouter *>( _routers[node] );

    // ---- Input buffers (Buffer::_size) ----
    {
      int port = 0;
      for ( int dim = 0; dim < n; ++dim ) {
        const int rn = right_nb( node, dim );
        const int ln = left_nb(  node, dim );

        // Channels arriving from rn (rn's left slot, dir=1)
        const int mult_rn = _NumParallel( node, rn );
        const int cnt_rn  = par_cnt[ rn * n * 2 + dim * 2 + 1 ];
        for ( int ch = 0; ch < cnt_rn; ++ch, ++port )
          rtr->GetInputBuffer( port )->SetSize( base_vc_buf * num_vcs * mult_rn );

        // Channels arriving from ln (ln's right slot, dir=0)
        const int mult_ln = _NumParallel( node, ln );
        const int cnt_ln  = par_cnt[ ln * n * 2 + dim * 2 + 0 ];
        for ( int ch = 0; ch < cnt_ln; ++ch, ++port )
          rtr->GetInputBuffer( port )->SetSize( base_vc_buf * num_vcs * mult_ln );
      }
      // injection ports follow — skip (port counter not advanced further)
    }

    // ---- Output buffer states (BufferState::_vc_buf_size) ----
    {
      int port = 0;
      for ( int dim = 0; dim < n; ++dim ) {
        const int rn = right_nb( node, dim );
        const int ln = left_nb(  node, dim );

        // Channels going to rn (node's right slot, dir=0)
        const int mult_rn = _NumParallel( node, rn );
        const int cnt_rn  = par_cnt[ node * n * 2 + dim * 2 + 0 ];
        for ( int ch = 0; ch < cnt_rn; ++ch, ++port )
          rtr->GetOutputBufferState( port )->SetVCBufSize( base_vc_buf * mult_rn );

        // Channels going to ln (node's left slot, dir=1)
        const int mult_ln = _NumParallel( node, ln );
        const int cnt_ln  = par_cnt[ node * n * 2 + dim * 2 + 1 ];
        for ( int ch = 0; ch < cnt_ln; ++ch, ++port )
          rtr->GetOutputBufferState( port )->SetVCBufSize( base_vc_buf * mult_ln );
      }
      // ejection port follows — skip
    }
  }
}

int Mesh::NumInjectionSlots( int source ) const
{
  return _NumInjectionSlots( source );
}

int Mesh::NumEjectionSlots( int dest ) const
{
  return _NumEjectionSlots( dest );
}

Flit * Mesh::ReadFlitFromEjectSlot( int dest, int slot ) const
{
  assert( dest >= 0 && dest < _nodes );
  assert( slot >= 0 && slot < _NumEjectionSlots( dest ) );
  if ( slot == 0 ) {
    return _eject[dest]->Receive();
  } else {
    return _eject_extra[dest][slot - 1]->Receive();
  }
}

Credit *Mesh::ReadCreditFromSlot( int source, int slot ) const
{
  assert( source >= 0 && source < _nodes );
  assert( slot >= 0 && slot < _NumInjectionSlots( source ) );
  if ( slot == 0 ) {
    return _inject_cred[source]->Receive();
  } else {
    return _inject_aux_cred[source][slot - 1]->Receive();
  }
}

bool Mesh::IsCrossNode( int node ) const
{
  assert( node >= 0 && node < _nodes );
  return _is_cross_node[node];
}

bool Mesh::IsCenterNode( int node ) const
{
  assert( node >= 0 && node < _nodes );
  return _is_center_node[node];
}

// ----------------------------------------------------------------
// WriteEjectCredit — send credit back on the correct ejection slot credit channel.
// ----------------------------------------------------------------
void Mesh::WriteEjectCredit( Credit *c, int dest, int slot )
{
  assert( dest >= 0 && dest < _nodes );
  assert( slot >= 0 && slot < _NumEjectionSlots( dest ) );
  if ( slot == 0 ) {
    _eject_cred[dest]->Send( c );
  } else {
    _eject_extra_cred[dest][slot - 1]->Send( c );
  }
}

// ----------------------------------------------------------------
// WriteFlit — dispatch to the correct NI slot for each node type.
//
//   Outer node : slot 0 = Px (is_px==true, XY routing)
//                slot 1 = Py (is_px==false, YX routing)
//   Cross node : slot 0 only
//   Center node: slot determined by dest direction
//                  0 = North (dest_row < center_row)
//                  1 = South (dest_row > center_row)
//                  2 = East  (dest_col > center_col, same row)
//                  3 = West  (dest_col < center_col, same row)
//                Slot is cached per pid so body/tail flits use
//                the same slot as the head flit.
// ----------------------------------------------------------------
void Mesh::WriteFlit( Flit *f, int source )
{
  assert( source >= 0 && source < _nodes );

  if ( IsCenterNode( source ) ) {
    int slot = 0;
    if ( f->head ) {
      const int cx = GetK() / 2;  // center col
      const int cy = GetK() / 2;  // center row
      const int dr = f->dest / GetK();
      const int dc = f->dest % GetK();
      if      ( dr < cy ) slot = 0;  // North
      else if ( dr > cy ) slot = 1;  // South
      else if ( dc > cx ) slot = 2;  // East
      else if ( dc < cx ) slot = 3;  // West
      else                slot = 0;  // self (fallback)
      _center_pkt_slot[f->pid] = slot;
    } else {
      auto it = _center_pkt_slot.find( f->pid );
      slot = ( it != _center_pkt_slot.end() ) ? it->second : 0;
    }
    if ( f->tail ) _center_pkt_slot.erase( f->pid );

    if ( slot == 0 ) {
      _inject[source]->Send( f );
    } else {
      _inject_aux[source][slot - 1]->Send( f );
    }

  } else if ( !IsCrossNode( source ) ) {
    // Outer node: Px → slot 0, Py → slot 1
    if ( f->is_px ) {
      _inject[source]->Send( f );
    } else {
      _inject_aux[source][0]->Send( f );
    }

  } else {
    // Cross node (not center): 2 NI slots (directional).
    // Phase 2 (→ center)  : slot 0.
    // Phase 4 col-cross   : slot 0 = west  (dest_col < half)
    //                       slot 1 = east  (dest_col > half)
    // Phase 4 row-cross   : slot 0 = north (dest_row < half)
    //                       slot 1 = south (dest_row > half)
    int slot = 0;
    if ( f->head ) {
      const int k       = GetK();
      const int half    = k / 2;
      const int src_col = source % k;

      if ( f->dest != -1 ) {
        const int center   = half * k + half;
        const int dest_row = f->dest / k;
        const int dest_col = f->dest % k;

        if ( f->dest == center ) {
          slot = 0;                                      // Phase 2: to center
        } else if ( src_col == half ) {
          slot = ( dest_col < half ) ? 0 : 1;           // col-cross: W=0, E=1
        } else {
          slot = ( dest_row < half ) ? 0 : 1;           // row-cross: N=0, S=1
        }
      }
      _cross_pkt_slot[f->pid] = slot;
    } else {
      auto it = _cross_pkt_slot.find( f->pid );
      slot = ( it != _cross_pkt_slot.end() ) ? it->second : 0;
    }
    if ( f->tail ) _cross_pkt_slot.erase( f->pid );

    if ( slot == 0 ) {
      _inject[source]->Send( f );
    } else {
      _inject_aux[source][0]->Send( f );
    }
  }
}

// ----------------------------------------------------------------
// ReadCredit — drain credit channels from all NI slots in order.
// Returns the first available credit; subsequent credits are
// returned in later cycles.
// ----------------------------------------------------------------
Credit *Mesh::ReadCredit( int source )
{
  assert( source >= 0 && source < _nodes );

  Credit *c = _inject_cred[source]->Receive();
  if ( c ) return c;

  for ( auto *cred_ch : _inject_aux_cred[source] ) {
    c = cred_ch->Receive();
    if ( c ) return c;
  }
  return nullptr;
}
