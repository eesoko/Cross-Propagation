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

/*mesh.hpp
 *
 * 2D Mesh topology with Fat Bus Cross node marking.
 * Extends KNCube (mesh=true) and annotates each node with:
 *   is_cross_node  — lies on center row OR center column
 *   is_center_node — lies on center row AND center column (single hub node)
 *
 * Requires n=2 (2D mesh). Asserts at construction for other values.
 */

#ifndef _MESH_HPP_
#define _MESH_HPP_

#include <map>
#include <vector>
#include "kncube.hpp"

// Global pointer to the active Mesh network, set during construction.
// Used by routing functions (e.g. cross_propagation_dor) to query node types
// without modifying the tRoutingFunction signature.
class Mesh;
extern Mesh *gMeshNet;

class Mesh : public KNCube {

  vector<bool> _is_cross_node;
  vector<bool> _is_center_node;

  void _MarkCrossNodes();

  // Returns number of parallel channels for a link between node_a and node_b.
  int _NumParallel( int node_a, int node_b ) const;

  // Returns number of injection NI slots for a node:
  //   outer node  → 2  (slot 0 = Px/XY, slot 1 = Py/YX)
  //   cross node  → 4  (slot 0-3 = 4 outer destinations for Phase 4)
  //   center node → 4  (slot 0=N, 1=S, 2=E, 3=W)
  int _NumInjectionSlots( int node ) const;

  // Returns number of ejection slots for a node:
  //   center node → 4  (slot 0=N, 1=S, 2=E, 3=W)
  //   all others  → 1
  int _NumEjectionSlots( int node ) const;

  // Extra injection NI channels (slots 1+) per node.
  // _inject_aux[node][s] corresponds to NI slot (s+1).
  vector<vector<FlitChannel *>>   _inject_aux;
  vector<vector<CreditChannel *>> _inject_aux_cred;

  // pid → NI slot cache for center and cross nodes (body/tail use same slot as head).
  mutable map<int, int> _center_pkt_slot;
  mutable map<int, int> _cross_pkt_slot;

  // Extra ejection output channels for center node (slots 1+).
  // _eject_extra[node][s] corresponds to ejection slot (s+1).
  vector<vector<FlitChannel *>>   _eject_extra;
  vector<vector<CreditChannel *>> _eject_extra_cred;

  // Overrides KNCube's _BuildNet to wire parallel (fat-bus) channels.
  void _BuildNet( const Configuration &config );

public:
  Mesh( const Configuration &config, const string &name );
  virtual ~Mesh();

  static void RegisterRoutingFunctions();

  // Returns true if node lies on center row or center column.
  bool IsCrossNode( int node ) const;

  // Returns true if node is the single hub at (center_row, center_col).
  bool IsCenterNode( int node ) const;

  // Returns the number of injection NI slots for source (1 / 2 / 4).
  int NumInjectionSlots( int source ) const;

  // Returns the number of ejection slots for dest (1 or 4 for center).
  int NumEjectionSlots( int dest ) const;

  // Read a credit from a specific NI slot (slot 0 = primary inject_cred,
  // slot 1+ = _inject_aux_cred[source][slot-1]).
  Credit * ReadCreditFromSlot( int source, int slot ) const;

  // Read a flit from a specific ejection slot (slot 0 = primary _eject,
  // slot 1+ = _eject_extra[dest][slot-1]).
  Flit * ReadFlitFromEjectSlot( int dest, int slot ) const;

  // Write a credit back to the correct ejection slot credit channel.
  // slot 0 → _eject_cred[dest], slot 1+ → _eject_extra_cred[dest][slot-1].
  void WriteEjectCredit( Credit *c, int dest, int slot );

  // Overrides: dispatch to the correct NI slot.
  void     WriteFlit  ( Flit   *f, int source ) override;
  Credit * ReadCredit ( int source )            override;
};

#endif
