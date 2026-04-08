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

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _TRAFFICMANAGER_HPP_
#define _TRAFFICMANAGER_HPP_

#include <list>
#include <map>
#include <set>
#include <cassert>

#include "module.hpp"
#include "config_utils.hpp"
#include "network.hpp"
#include "flit.hpp"
#include "buffer_state.hpp"
#include "stats.hpp"
#include "traffic.hpp"
#include "routefunc.hpp"
#include "outputset.hpp"
#include "injection.hpp"

//register the requests to a node
class PacketReplyInfo;

class TrafficManager : public Module {

private:

  vector<vector<int> > _packet_size;
  vector<vector<int> > _packet_size_rate;
  vector<int> _packet_size_max_val;

protected:
  int _nodes;
  int _routers;
  int _vcs;

  vector<Network *> _net;
  vector<vector<Router *> > _router;

  // ============ Traffic ============ 

  int    _classes;

  vector<double> _load;

  vector<int> _use_read_write;
  vector<double> _write_fraction;

  vector<int> _read_request_size;
  vector<int> _read_reply_size;
  vector<int> _write_request_size;
  vector<int> _write_reply_size;

  vector<string> _traffic;

  vector<int> _class_priority;

  vector<vector<vector<int> > > _last_class;  // [node][slot][subnet]

  vector<TrafficPattern *> _traffic_pattern;
  vector<InjectionProcess *> _injection_process;

  // ============ Message priorities ============ 

  enum ePriority { class_based, age_based, network_age_based, local_age_based, queue_length_based, hop_count_based, sequence_based, none };

  ePriority _pri_type;

  // ============ Injection VC states  ============ 

  vector<vector<vector<BufferState *> > > _buf_states;  // [node][slot][subnet]
#ifdef TRACK_FLOWS
  vector<vector<vector<int> > > _outstanding_credits;
  vector<vector<vector<queue<int> > > > _outstanding_classes;
#endif
  vector<vector<vector<vector<int> > > > _last_vc;  // [node][slot][subnet][class]

  // ============ Routing ============ 

  tRoutingFunction _rf;
  bool _lookahead_routing;
  bool _noq;

  // ============ Injection queues ============ 

  vector<vector<int> > _qtime;
  vector<vector<bool> > _qdrained;
  vector<vector<vector<list<Flit *> > > > _partial_packets;  // [node][slot][class]

  vector<map<int, Flit *> > _total_in_flight_flits;
  vector<map<int, Flit *> > _measured_in_flight_flits;
  vector<map<int, Flit *> > _retired_packets;
  bool _empty_network;

  bool _hold_switch_for_packet;

  // ============ physical sub-networks ==========

  int _subnets;

  vector<int> _subnet;

  // ============ deadlock ==========

  int _deadlock_timer;
  int _deadlock_warn_timeout;

  // ============ request & replies ==========================

  vector<int> _packet_seq_no;
  vector<list<PacketReplyInfo*> > _repliesPending;
  vector<int> _requestsOutstanding;

  // ============ Statistics ============

  vector<Stats *> _plat_stats;     
  vector<double> _overall_min_plat;  
  vector<double> _overall_avg_plat;  
  vector<double> _overall_max_plat;  

  vector<Stats *> _nlat_stats;     
  vector<double> _overall_min_nlat;  
  vector<double> _overall_avg_nlat;  
  vector<double> _overall_max_nlat;  

  vector<Stats *> _flat_stats;     
  vector<double> _overall_min_flat;  
  vector<double> _overall_avg_flat;  
  vector<double> _overall_max_flat;  

  vector<Stats *> _frag_stats;
  vector<double> _overall_min_frag;
  vector<double> _overall_avg_frag;
  vector<double> _overall_max_frag;

  vector<vector<Stats *> > _pair_plat;
  vector<vector<Stats *> > _pair_nlat;
  vector<vector<Stats *> > _pair_flat;

  vector<Stats *> _hop_stats;
  vector<double> _overall_hop_stats;

  vector<vector<int> > _sent_packets;
  vector<double> _overall_min_sent_packets;
  vector<double> _overall_avg_sent_packets;
  vector<double> _overall_max_sent_packets;
  vector<vector<int> > _accepted_packets;
  vector<double> _overall_min_accepted_packets;
  vector<double> _overall_avg_accepted_packets;
  vector<double> _overall_max_accepted_packets;
  vector<vector<int> > _sent_flits;
  vector<double> _overall_min_sent;
  vector<double> _overall_avg_sent;
  vector<double> _overall_max_sent;
  vector<vector<int> > _accepted_flits;
  vector<double> _overall_min_accepted;
  vector<double> _overall_avg_accepted;
  vector<double> _overall_max_accepted;

#ifdef TRACK_STALLS
  vector<vector<int> > _buffer_busy_stalls;
  vector<vector<int> > _buffer_conflict_stalls;
  vector<vector<int> > _buffer_full_stalls;
  vector<vector<int> > _buffer_reserved_stalls;
  vector<vector<int> > _crossbar_conflict_stalls;
  vector<double> _overall_buffer_busy_stalls;
  vector<double> _overall_buffer_conflict_stalls;
  vector<double> _overall_buffer_full_stalls;
  vector<double> _overall_buffer_reserved_stalls;
  vector<double> _overall_crossbar_conflict_stalls;
#endif

  vector<int> _slowest_packet;
  vector<int> _slowest_flit;

  map<string, Stats *> _stats;

  // ============ Simulation parameters ============ 

  enum eSimState { warming_up, running, draining, done };
  eSimState _sim_state;

  bool _measure_latency;

  int   _reset_time;
  int   _drain_time;

  int   _total_sims;
  int   _sample_period;
  int   _max_samples;
  int   _warmup_periods;

  int   _include_queuing;

  vector<int> _measure_stats;
  bool _pair_stats;

  vector<double> _latency_thres;

  vector<double> _stopping_threshold;
  vector<double> _acc_stopping_threshold;

  vector<double> _warmup_threshold;
  vector<double> _acc_warmup_threshold;

  int _cur_id;
  int _cur_pid;
  int _time;

  set<int> _flits_to_watch;
  set<int> _packets_to_watch;

  bool _print_csv_results;

  //flits to watch
  ostream * _stats_out;

#ifdef TRACK_FLOWS
  vector<vector<int> > _injected_flits;
  vector<vector<int> > _ejected_flits;
  ostream * _injected_flits_out;
  ostream * _received_flits_out;
  ostream * _stored_flits_out;
  ostream * _sent_flits_out;
  ostream * _outstanding_credits_out;
  ostream * _ejected_flits_out;
  ostream * _active_packets_out;
#endif

#ifdef TRACK_CREDITS
  ostream * _used_credits_out;
  ostream * _free_credits_out;
  ostream * _max_credits_out;
#endif

  // ============ Internal methods ============ 
protected:

  virtual void _RetireFlit( Flit *f, int dest );

  void _Inject();
  void _Step( );

  bool _PacketsOutstanding( ) const;

  virtual int  _IssuePacket( int source, int cl );
  void _GeneratePacket( int source, int size, int cl, int time );

  // ============ Cross-Propagation All-Reduce (Hierarchical 4-Phase) ============
  //
  // State machine:
  //   _cp_phase == 0 : Phase 1 – Outer → Cross  (local barrier per cross node)
  //                    Phase 2 – Cross → Center  (global barrier at center)
  //   _cp_phase == 1 : Phase 3 – Center → 8 Cross
  //                    Phase 4 – Col-Cross → Outer (D/2 Px, horizontal)
  //                              Row-Cross → Outer (D/2 Py, vertical)
  //   _cp_phase == 2 : done
  //
  // Cross node index mapping (0-7):
  //   col-cross : N(r, k/2) → index r  for r in {0,1,3,4}  (0-based rows, skipping center)
  //   row-cross : N(k/2, c) → index 4+ for c in {0,1,3,4}
  //   Concretely for k=5: N2→0, N7→1, N17→2, N22→3, N10→4, N11→5, N13→6, N14→7

  // true when traffic pattern is "cross_propagation"
  bool _cp_enabled;
  // true after the one-shot Phase 0 bulk injection (outer→cross) has fired
  bool _cp_phase1_injected;
  // center node ID = (k/2)*k + (k/2)
  int  _cp_center;
  // mesh dimension k (cached from gK)
  int  _cp_k;
  // current phase: 0 = reduce in progress, 1 = broadcast in progress, 2 = done
  int  _cp_phase;

  // --- Phase 0 local barriers (one per cross node, index 0-7) ---
  // Number of tail packets received at each cross node from outer nodes.
  // Threshold = 4 (each cross node has 4 assigned outer nodes).
  int  _cp_cross_recv[8];
  // Whether this cross node has already injected its aggregated packet to center.
  bool _cp_cross_sent[8];

  // --- Phase 0.5 global barrier (at center) ---
  // Number of tail packets received at center from cross nodes. Threshold = 8.
  int  _cp_center_recv;

  // --- Phase 1 sub-barriers (one per cross node) ---
  // Whether each cross node has received the broadcast packet from center.
  bool _cp_cross_p2_recv[8];

  // --- Phase 1.5 end barrier ---
  // Number of tail packets received at outer nodes from col-cross nodes. Threshold = 16.
  int  _cp_outer_recv;

  // --- Timing ---
  int  _cp_phase1_start_cycle;   // cycle when Phase 0 injection fired
  int  _cp_phase1_end_cycle;     // cycle when center global barrier fired
  int  _cp_phase2_end_cycle;     // cycle when last outer node received Phase 1.5 packet

  // --- Traffic volume ---
  int  _cp_total_flits;          // total flits injected across all CP phases

  // --- Node classification helpers ---
  // Returns true if node is a col-cross node (on center col, not center row).
  bool _CpIsColCross( int node ) const;
  // Returns true if node is a row-cross node (on center row, not center col).
  bool _CpIsRowCross( int node ) const;
  // Returns cross-node array index (0-7) for a cross (non-center) node.
  int  _CpCrossIndex( int node ) const;
  // Returns the col-cross node ID for a given row  (= row * k + k/2).
  int  _CpColCrossForRow( int row ) const;
  // Returns the row-cross node ID for a given col  (= (k/2)*k + col).
  int  _CpRowCrossForCol( int col ) const;

  // --- Injection helpers ---
  // Enqueue one packet with explicit src/dest/size/routing flag.
  void _CpInjectPacket( int src, int dest, int size, int cl, bool is_px );
  // Phase 0  : enqueue all outer-node Px/Py packets (outer → cross).
  void _InjectAllPhase1();
  // Phase 0.5: cross node injects its aggregated sum to center.
  void _InjectCrossToCenter( int cross_node );
  // Phase 1  : center injects broadcast packets to all 8 cross nodes.
  void _InjectCenterToCross();
  // Phase 4a: col-cross node injects D/2 Px packets to its 4 outer nodes (horizontal).
  void _InjectColCrossToOuter( int col_cross_node );
  // Phase 4b: row-cross node injects D/2 Py packets to its 4 outer nodes (vertical).
  void _InjectRowCrossToOuter( int row_cross_node );

  virtual void _ClearStats( );

  void _ComputeStats( const vector<int> & stats, int *sum, int *min = NULL, int *max = NULL, int *min_pos = NULL, int *max_pos = NULL ) const;

  virtual bool _SingleSim( );

  void _DisplayRemaining( ostream & os = cout ) const;
  
  void _LoadWatchList(const string & filename);

  virtual void _UpdateOverallStats();

  virtual string _OverallStatsCSV(int c = 0) const;

  int _GetNextPacketSize(int cl) const;
  double _GetAveragePacketSize(int cl) const;

public:

  static TrafficManager * New(Configuration const & config, 
			      vector<Network *> const & net);

  TrafficManager( const Configuration &config, const vector<Network *> & net );
  virtual ~TrafficManager( );

  bool Run( );

  virtual void WriteStats( ostream & os = cout ) const ;
  virtual void UpdateStats( ) ;
  virtual void DisplayStats( ostream & os = cout ) const ;
  virtual void DisplayOverallStats( ostream & os = cout ) const ;
  virtual void DisplayOverallStatsCSV( ostream & os = cout ) const ;

  inline int getTime() { return _time;}
  Stats * getStats(const string & name) { return _stats[name]; }

};

template<class T>
ostream & operator<<(ostream & os, const vector<T> & v) {
  for(size_t i = 0; i < v.size() - 1; ++i) {
    os << v[i] << ",";
  }
  os << v[v.size()-1];
  return os;
}

#endif
