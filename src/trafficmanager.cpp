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

#include <sstream>
#include <cmath>
#include <fstream>
#include <limits>
#include <cstdlib>
#include <ctime>

#include "booksim.hpp"
#include "booksim_config.hpp"
#include "trafficmanager.hpp"
#include "batchtrafficmanager.hpp"
#include "random_utils.hpp"
#include "vc.hpp"
#include "packet_reply_info.hpp"
#include "mesh.hpp"

TrafficManager * TrafficManager::New(Configuration const & config,
                                     vector<Network *> const & net)
{
    TrafficManager * result = NULL;
    string sim_type = config.GetStr("sim_type");
    if((sim_type == "latency") || (sim_type == "throughput")) {
        result = new TrafficManager(config, net);
    } else if(sim_type == "batch") {
        result = new BatchTrafficManager(config, net);
    } else {
        cerr << "Unknown simulation type: " << sim_type << endl;
    } 
    return result;
}

TrafficManager::TrafficManager( const Configuration &config, const vector<Network *> & net )
    : Module( 0, "traffic_manager" ), _net(net), _empty_network(false), _deadlock_timer(0), _reset_time(0), _drain_time(-1), _cur_id(0), _cur_pid(0), _time(0)
{

    _nodes = _net[0]->NumNodes( );
    _routers = _net[0]->NumRouters( );

    _vcs = config.GetInt("num_vcs");
    _subnets = config.GetInt("subnets");
 
    _subnet.resize(Flit::NUM_FLIT_TYPES);
    _subnet[Flit::READ_REQUEST] = config.GetInt("read_request_subnet");
    _subnet[Flit::READ_REPLY] = config.GetInt("read_reply_subnet");
    _subnet[Flit::WRITE_REQUEST] = config.GetInt("write_request_subnet");
    _subnet[Flit::WRITE_REPLY] = config.GetInt("write_reply_subnet");

    // ============ Message priorities ============ 

    string priority = config.GetStr( "priority" );

    if ( priority == "class" ) {
        _pri_type = class_based;
    } else if ( priority == "age" ) {
        _pri_type = age_based;
    } else if ( priority == "network_age" ) {
        _pri_type = network_age_based;
    } else if ( priority == "local_age" ) {
        _pri_type = local_age_based;
    } else if ( priority == "queue_length" ) {
        _pri_type = queue_length_based;
    } else if ( priority == "hop_count" ) {
        _pri_type = hop_count_based;
    } else if ( priority == "sequence" ) {
        _pri_type = sequence_based;
    } else if ( priority == "none" ) {
        _pri_type = none;
    } else {
        Error( "Unkown priority value: " + priority );
    }

    // ============ Routing ============ 

    string rf = config.GetStr("routing_function") + "_" + config.GetStr("topology");
    map<string, tRoutingFunction>::const_iterator rf_iter = gRoutingFunctionMap.find(rf);
    if(rf_iter == gRoutingFunctionMap.end()) {
        Error("Invalid routing function: " + rf);
    }
    _rf = rf_iter->second;
  
    _lookahead_routing = !config.GetInt("routing_delay");
    _noq = config.GetInt("noq");
    if(_noq) {
        if(!_lookahead_routing) {
            Error("NOQ requires lookahead routing to be enabled.");
        }
    }

    // ============ Traffic ============ 

    _classes = config.GetInt("classes");

    _use_read_write = config.GetIntArray("use_read_write");
    if(_use_read_write.empty()) {
        _use_read_write.push_back(config.GetInt("use_read_write"));
    }
    _use_read_write.resize(_classes, _use_read_write.back());

    _write_fraction = config.GetFloatArray("write_fraction");
    if(_write_fraction.empty()) {
        _write_fraction.push_back(config.GetFloat("write_fraction"));
    }
    _write_fraction.resize(_classes, _write_fraction.back());

    _read_request_size = config.GetIntArray("read_request_size");
    if(_read_request_size.empty()) {
        _read_request_size.push_back(config.GetInt("read_request_size"));
    }
    _read_request_size.resize(_classes, _read_request_size.back());

    _read_reply_size = config.GetIntArray("read_reply_size");
    if(_read_reply_size.empty()) {
        _read_reply_size.push_back(config.GetInt("read_reply_size"));
    }
    _read_reply_size.resize(_classes, _read_reply_size.back());

    _write_request_size = config.GetIntArray("write_request_size");
    if(_write_request_size.empty()) {
        _write_request_size.push_back(config.GetInt("write_request_size"));
    }
    _write_request_size.resize(_classes, _write_request_size.back());

    _write_reply_size = config.GetIntArray("write_reply_size");
    if(_write_reply_size.empty()) {
        _write_reply_size.push_back(config.GetInt("write_reply_size"));
    }
    _write_reply_size.resize(_classes, _write_reply_size.back());

    string packet_size_str = config.GetStr("packet_size");
    if(packet_size_str.empty()) {
        _packet_size.push_back(vector<int>(1, config.GetInt("packet_size")));
    } else {
        vector<string> packet_size_strings = tokenize_str(packet_size_str);
        for(size_t i = 0; i < packet_size_strings.size(); ++i) {
            _packet_size.push_back(tokenize_int(packet_size_strings[i]));
        }
    }
    _packet_size.resize(_classes, _packet_size.back());

    string packet_size_rate_str = config.GetStr("packet_size_rate");
    if(packet_size_rate_str.empty()) {
        int rate = config.GetInt("packet_size_rate");
        assert(rate >= 0);
        for(int c = 0; c < _classes; ++c) {
            int size = _packet_size[c].size();
            _packet_size_rate.push_back(vector<int>(size, rate));
            _packet_size_max_val.push_back(size * rate - 1);
        }
    } else {
        vector<string> packet_size_rate_strings = tokenize_str(packet_size_rate_str);
        packet_size_rate_strings.resize(_classes, packet_size_rate_strings.back());
        for(int c = 0; c < _classes; ++c) {
            vector<int> rates = tokenize_int(packet_size_rate_strings[c]);
            rates.resize(_packet_size[c].size(), rates.back());
            _packet_size_rate.push_back(rates);
            int size = rates.size();
            int max_val = -1;
            for(int i = 0; i < size; ++i) {
                int rate = rates[i];
                assert(rate >= 0);
                max_val += rate;
            }
            _packet_size_max_val.push_back(max_val);
        }
    }
  
    for(int c = 0; c < _classes; ++c) {
        if(_use_read_write[c]) {
            _packet_size[c] = 
                vector<int>(1, (_read_request_size[c] + _read_reply_size[c] +
                                _write_request_size[c] + _write_reply_size[c]) / 2);
            _packet_size_rate[c] = vector<int>(1, 1);
            _packet_size_max_val[c] = 0;
        }
    }

    _load = config.GetFloatArray("injection_rate"); 
    if(_load.empty()) {
        _load.push_back(config.GetFloat("injection_rate"));
    }
    _load.resize(_classes, _load.back());

    if(config.GetInt("injection_rate_uses_flits")) {
        for(int c = 0; c < _classes; ++c)
            _load[c] /= _GetAveragePacketSize(c);
    }

    _traffic = config.GetStrArray("traffic");
    _traffic.resize(_classes, _traffic.back());

    _traffic_pattern.resize(_classes);

    _class_priority = config.GetIntArray("class_priority"); 
    if(_class_priority.empty()) {
        _class_priority.push_back(config.GetInt("class_priority"));
    }
    _class_priority.resize(_classes, _class_priority.back());

    vector<string> injection_process = config.GetStrArray("injection_process");
    injection_process.resize(_classes, injection_process.back());

    _injection_process.resize(_classes);

    for(int c = 0; c < _classes; ++c) {
        _traffic_pattern[c] = TrafficPattern::New(_traffic[c], _nodes, &config);
        _injection_process[c] = InjectionProcess::New(injection_process[c], _nodes, _load[c], &config);
    }

    // ============ Injection VC states  ============

    _buf_states.resize(_nodes);
    _last_vc.resize(_nodes);
    _last_class.resize(_nodes);

    const Mesh * mesh_init = dynamic_cast<const Mesh *>(_net[0]);

    for ( int source = 0; source < _nodes; ++source ) {
        int ni = mesh_init ? mesh_init->NumInjectionSlots(source) : 1;
        _buf_states[source].resize(ni);
        _last_class[source].resize(ni);
        _last_vc[source].resize(ni);
        for ( int slot = 0; slot < ni; ++slot ) {
            _buf_states[source][slot].resize(_subnets);
            _last_class[source][slot].resize(_subnets, 0);
            _last_vc[source][slot].resize(_subnets);
            for ( int subnet = 0; subnet < _subnets; ++subnet ) {
                ostringstream tmp_name;
                tmp_name << "terminal_buf_state_" << source << "_" << slot << "_" << subnet;
                BufferState * bs = new BufferState( config, this, tmp_name.str( ) );
                int vc_alloc_delay = config.GetInt("vc_alloc_delay");
                int sw_alloc_delay = config.GetInt("sw_alloc_delay");
                int router_latency = config.GetInt("routing_delay") + (config.GetInt("speculative") ? max(vc_alloc_delay, sw_alloc_delay) : (vc_alloc_delay + sw_alloc_delay));
                int min_latency = 1 + _net[subnet]->GetInject(source)->GetLatency() + router_latency + _net[subnet]->GetInjectCred(source)->GetLatency();
                bs->SetMinLatency(min_latency);
                _buf_states[source][slot][subnet] = bs;
                _last_vc[source][slot][subnet].resize(_classes, -1);
            }
        }
    }

#ifdef TRACK_FLOWS
    _outstanding_credits.resize(_classes);
    for(int c = 0; c < _classes; ++c) {
        _outstanding_credits[c].resize(_subnets, vector<int>(_nodes, 0));
    }
    _outstanding_classes.resize(_nodes);
    for(int n = 0; n < _nodes; ++n) {
        _outstanding_classes[n].resize(_subnets, vector<queue<int> >(_vcs));
    }
#endif

    // ============ Injection queues ============ 

    _qtime.resize(_nodes);
    _qdrained.resize(_nodes);
    _partial_packets.resize(_nodes);

    for ( int s = 0; s < _nodes; ++s ) {
        _qtime[s].resize(_classes);
        _qdrained[s].resize(_classes);
        int ni = mesh_init ? mesh_init->NumInjectionSlots(s) : 1;
        _partial_packets[s].resize(ni);
        for ( int slot = 0; slot < ni; ++slot ) {
            _partial_packets[s][slot].resize(_classes);
        }
    }

    _total_in_flight_flits.resize(_classes);
    _measured_in_flight_flits.resize(_classes);
    _retired_packets.resize(_classes);

    _packet_seq_no.resize(_nodes);
    _repliesPending.resize(_nodes);
    _requestsOutstanding.resize(_nodes);

    _hold_switch_for_packet = config.GetInt("hold_switch_for_packet");

    // ============ Simulation parameters ============ 

    _total_sims = config.GetInt( "sim_count" );

    _router.resize(_subnets);
    for (int i=0; i < _subnets; ++i) {
        _router[i] = _net[i]->GetRouters();
    }

    //seed the network
    int seed;
    if(config.GetStr("seed") == "time") {
      seed = int(time(NULL));
      cout << "SEED: seed=" << seed << endl;
    } else {
      seed = config.GetInt("seed");
    }
    RandomSeed(seed);

    _measure_latency = (config.GetStr("sim_type") == "latency");

    _sample_period = config.GetInt( "sample_period" );
    _max_samples    = config.GetInt( "max_samples" );
    _warmup_periods = config.GetInt( "warmup_periods" );

    _measure_stats = config.GetIntArray( "measure_stats" );
    if(_measure_stats.empty()) {
        _measure_stats.push_back(config.GetInt("measure_stats"));
    }
    _measure_stats.resize(_classes, _measure_stats.back());
    _pair_stats = (config.GetInt("pair_stats")==1);

    _latency_thres = config.GetFloatArray( "latency_thres" );
    if(_latency_thres.empty()) {
        _latency_thres.push_back(config.GetFloat("latency_thres"));
    }
    _latency_thres.resize(_classes, _latency_thres.back());

    _warmup_threshold = config.GetFloatArray( "warmup_thres" );
    if(_warmup_threshold.empty()) {
        _warmup_threshold.push_back(config.GetFloat("warmup_thres"));
    }
    _warmup_threshold.resize(_classes, _warmup_threshold.back());

    _acc_warmup_threshold = config.GetFloatArray( "acc_warmup_thres" );
    if(_acc_warmup_threshold.empty()) {
        _acc_warmup_threshold.push_back(config.GetFloat("acc_warmup_thres"));
    }
    _acc_warmup_threshold.resize(_classes, _acc_warmup_threshold.back());

    _stopping_threshold = config.GetFloatArray( "stopping_thres" );
    if(_stopping_threshold.empty()) {
        _stopping_threshold.push_back(config.GetFloat("stopping_thres"));
    }
    _stopping_threshold.resize(_classes, _stopping_threshold.back());

    _acc_stopping_threshold = config.GetFloatArray( "acc_stopping_thres" );
    if(_acc_stopping_threshold.empty()) {
        _acc_stopping_threshold.push_back(config.GetFloat("acc_stopping_thres"));
    }
    _acc_stopping_threshold.resize(_classes, _acc_stopping_threshold.back());

    _include_queuing = config.GetInt( "include_queuing" );

    _print_csv_results = config.GetInt( "print_csv_results" );
    _deadlock_warn_timeout = config.GetInt( "deadlock_warn_timeout" );

    string watch_file = config.GetStr( "watch_file" );
    if((watch_file != "") && (watch_file != "-")) {
        _LoadWatchList(watch_file);
    }

    vector<int> watch_flits = config.GetIntArray("watch_flits");
    for(size_t i = 0; i < watch_flits.size(); ++i) {
        _flits_to_watch.insert(watch_flits[i]);
    }
  
    vector<int> watch_packets = config.GetIntArray("watch_packets");
    for(size_t i = 0; i < watch_packets.size(); ++i) {
        _packets_to_watch.insert(watch_packets[i]);
    }

    string stats_out_file = config.GetStr( "stats_out" );
    if(stats_out_file == "") {
        _stats_out = NULL;
    } else if(stats_out_file == "-") {
        _stats_out = &cout;
    } else {
        _stats_out = new ofstream(stats_out_file.c_str());
        config.WriteMatlabFile(_stats_out);
    }
  
#ifdef TRACK_FLOWS
    _injected_flits.resize(_classes, vector<int>(_nodes, 0));
    _ejected_flits.resize(_classes, vector<int>(_nodes, 0));
    string injected_flits_out_file = config.GetStr( "injected_flits_out" );
    if(injected_flits_out_file == "") {
        _injected_flits_out = NULL;
    } else {
        _injected_flits_out = new ofstream(injected_flits_out_file.c_str());
    }
    string received_flits_out_file = config.GetStr( "received_flits_out" );
    if(received_flits_out_file == "") {
        _received_flits_out = NULL;
    } else {
        _received_flits_out = new ofstream(received_flits_out_file.c_str());
    }
    string stored_flits_out_file = config.GetStr( "stored_flits_out" );
    if(stored_flits_out_file == "") {
        _stored_flits_out = NULL;
    } else {
        _stored_flits_out = new ofstream(stored_flits_out_file.c_str());
    }
    string sent_flits_out_file = config.GetStr( "sent_flits_out" );
    if(sent_flits_out_file == "") {
        _sent_flits_out = NULL;
    } else {
        _sent_flits_out = new ofstream(sent_flits_out_file.c_str());
    }
    string outstanding_credits_out_file = config.GetStr( "outstanding_credits_out" );
    if(outstanding_credits_out_file == "") {
        _outstanding_credits_out = NULL;
    } else {
        _outstanding_credits_out = new ofstream(outstanding_credits_out_file.c_str());
    }
    string ejected_flits_out_file = config.GetStr( "ejected_flits_out" );
    if(ejected_flits_out_file == "") {
        _ejected_flits_out = NULL;
    } else {
        _ejected_flits_out = new ofstream(ejected_flits_out_file.c_str());
    }
    string active_packets_out_file = config.GetStr( "active_packets_out" );
    if(active_packets_out_file == "") {
        _active_packets_out = NULL;
    } else {
        _active_packets_out = new ofstream(active_packets_out_file.c_str());
    }
#endif

#ifdef TRACK_CREDITS
    string used_credits_out_file = config.GetStr( "used_credits_out" );
    if(used_credits_out_file == "") {
        _used_credits_out = NULL;
    } else {
        _used_credits_out = new ofstream(used_credits_out_file.c_str());
    }
    string free_credits_out_file = config.GetStr( "free_credits_out" );
    if(free_credits_out_file == "") {
        _free_credits_out = NULL;
    } else {
        _free_credits_out = new ofstream(free_credits_out_file.c_str());
    }
    string max_credits_out_file = config.GetStr( "max_credits_out" );
    if(max_credits_out_file == "") {
        _max_credits_out = NULL;
    } else {
        _max_credits_out = new ofstream(max_credits_out_file.c_str());
    }
#endif

    // ============ Statistics ============ 

    _plat_stats.resize(_classes);
    _overall_min_plat.resize(_classes, 0.0);
    _overall_avg_plat.resize(_classes, 0.0);
    _overall_max_plat.resize(_classes, 0.0);

    _nlat_stats.resize(_classes);
    _overall_min_nlat.resize(_classes, 0.0);
    _overall_avg_nlat.resize(_classes, 0.0);
    _overall_max_nlat.resize(_classes, 0.0);

    _flat_stats.resize(_classes);
    _overall_min_flat.resize(_classes, 0.0);
    _overall_avg_flat.resize(_classes, 0.0);
    _overall_max_flat.resize(_classes, 0.0);

    _frag_stats.resize(_classes);
    _overall_min_frag.resize(_classes, 0.0);
    _overall_avg_frag.resize(_classes, 0.0);
    _overall_max_frag.resize(_classes, 0.0);

    if(_pair_stats){
        _pair_plat.resize(_classes);
        _pair_nlat.resize(_classes);
        _pair_flat.resize(_classes);
    }
  
    _hop_stats.resize(_classes);
    _overall_hop_stats.resize(_classes, 0.0);
  
    _sent_packets.resize(_classes);
    _overall_min_sent_packets.resize(_classes, 0.0);
    _overall_avg_sent_packets.resize(_classes, 0.0);
    _overall_max_sent_packets.resize(_classes, 0.0);
    _accepted_packets.resize(_classes);
    _overall_min_accepted_packets.resize(_classes, 0.0);
    _overall_avg_accepted_packets.resize(_classes, 0.0);
    _overall_max_accepted_packets.resize(_classes, 0.0);

    _sent_flits.resize(_classes);
    _overall_min_sent.resize(_classes, 0.0);
    _overall_avg_sent.resize(_classes, 0.0);
    _overall_max_sent.resize(_classes, 0.0);
    _accepted_flits.resize(_classes);
    _overall_min_accepted.resize(_classes, 0.0);
    _overall_avg_accepted.resize(_classes, 0.0);
    _overall_max_accepted.resize(_classes, 0.0);

#ifdef TRACK_STALLS
    _buffer_busy_stalls.resize(_classes);
    _buffer_conflict_stalls.resize(_classes);
    _buffer_full_stalls.resize(_classes);
    _buffer_reserved_stalls.resize(_classes);
    _crossbar_conflict_stalls.resize(_classes);
    _overall_buffer_busy_stalls.resize(_classes, 0);
    _overall_buffer_conflict_stalls.resize(_classes, 0);
    _overall_buffer_full_stalls.resize(_classes, 0);
    _overall_buffer_reserved_stalls.resize(_classes, 0);
    _overall_crossbar_conflict_stalls.resize(_classes, 0);
#endif

    for ( int c = 0; c < _classes; ++c ) {
        ostringstream tmp_name;

        tmp_name << "plat_stat_" << c;
        _plat_stats[c] = new Stats( this, tmp_name.str( ), 1.0, 1000 );
        _stats[tmp_name.str()] = _plat_stats[c];
        tmp_name.str("");

        tmp_name << "nlat_stat_" << c;
        _nlat_stats[c] = new Stats( this, tmp_name.str( ), 1.0, 1000 );
        _stats[tmp_name.str()] = _nlat_stats[c];
        tmp_name.str("");

        tmp_name << "flat_stat_" << c;
        _flat_stats[c] = new Stats( this, tmp_name.str( ), 1.0, 1000 );
        _stats[tmp_name.str()] = _flat_stats[c];
        tmp_name.str("");

        tmp_name << "frag_stat_" << c;
        _frag_stats[c] = new Stats( this, tmp_name.str( ), 1.0, 100 );
        _stats[tmp_name.str()] = _frag_stats[c];
        tmp_name.str("");

        tmp_name << "hop_stat_" << c;
        _hop_stats[c] = new Stats( this, tmp_name.str( ), 1.0, 20 );
        _stats[tmp_name.str()] = _hop_stats[c];
        tmp_name.str("");

        if(_pair_stats){
            _pair_plat[c].resize(_nodes*_nodes);
            _pair_nlat[c].resize(_nodes*_nodes);
            _pair_flat[c].resize(_nodes*_nodes);
        }

        _sent_packets[c].resize(_nodes, 0);
        _accepted_packets[c].resize(_nodes, 0);
        _sent_flits[c].resize(_nodes, 0);
        _accepted_flits[c].resize(_nodes, 0);

#ifdef TRACK_STALLS
        _buffer_busy_stalls[c].resize(_subnets*_routers, 0);
        _buffer_conflict_stalls[c].resize(_subnets*_routers, 0);
        _buffer_full_stalls[c].resize(_subnets*_routers, 0);
        _buffer_reserved_stalls[c].resize(_subnets*_routers, 0);
        _crossbar_conflict_stalls[c].resize(_subnets*_routers, 0);
#endif
        if(_pair_stats){
            for ( int i = 0; i < _nodes; ++i ) {
                for ( int j = 0; j < _nodes; ++j ) {
                    tmp_name << "pair_plat_stat_" << c << "_" << i << "_" << j;
                    _pair_plat[c][i*_nodes+j] = new Stats( this, tmp_name.str( ), 1.0, 250 );
                    _stats[tmp_name.str()] = _pair_plat[c][i*_nodes+j];
                    tmp_name.str("");
	  
                    tmp_name << "pair_nlat_stat_" << c << "_" << i << "_" << j;
                    _pair_nlat[c][i*_nodes+j] = new Stats( this, tmp_name.str( ), 1.0, 250 );
                    _stats[tmp_name.str()] = _pair_nlat[c][i*_nodes+j];
                    tmp_name.str("");
	  
                    tmp_name << "pair_flat_stat_" << c << "_" << i << "_" << j;
                    _pair_flat[c][i*_nodes+j] = new Stats( this, tmp_name.str( ), 1.0, 250 );
                    _stats[tmp_name.str()] = _pair_flat[c][i*_nodes+j];
                    tmp_name.str("");
                }
            }
        }
    }

    _slowest_flit.resize(_classes, -1);
    _slowest_packet.resize(_classes, -1);

    // ============ Cross-Propagation All-Reduce init ============
    _cp_enabled = false;
    for(int c = 0; c < _classes; ++c) {
        if(_traffic[c] == "cross_propagation") {
            _cp_enabled = true;
            break;
        }
    }
    _cp_phase1_injected        = false;
    _cp_k                      = gK;
    _cp_center                 = (_cp_k / 2) * _cp_k + (_cp_k / 2);
    _cp_phase                  = 0;
    _cp_center_recv            = 0;
    _cp_outer_recv             = 0;
    _cp_phase1_start_cycle     = -1;
    _cp_phase1_local_end_cycle = -1;
    _cp_phase2_end_cycle       = -1;
    _cp_phase3_end_cycle       = -1;
    _cp_phase4_end_cycle       = -1;
    _cp_cross_all_sent_count   = 0;
    _cp_cross_p2_recv_count    = 0;
    _cp_total_flits            = 0;
    _cp_trace = (config.GetInt("cp_trace") != 0);
    for ( int i = 0; i < 8; ++i ) {
        _cp_cross_recv[i]    = 0;
        _cp_cross_sent[i]    = false;
        _cp_cross_p2_recv[i] = false;
    }

}

TrafficManager::~TrafficManager( )
{

    for ( int source = 0; source < _nodes; ++source ) {
        for ( int slot = 0; slot < (int)_buf_states[source].size(); ++slot ) {
            for ( int subnet = 0; subnet < _subnets; ++subnet ) {
                delete _buf_states[source][slot][subnet];
            }
        }
    }
  
    for ( int c = 0; c < _classes; ++c ) {
        delete _plat_stats[c];
        delete _nlat_stats[c];
        delete _flat_stats[c];
        delete _frag_stats[c];
        delete _hop_stats[c];

        delete _traffic_pattern[c];
        delete _injection_process[c];
        if(_pair_stats){
            for ( int i = 0; i < _nodes; ++i ) {
                for ( int j = 0; j < _nodes; ++j ) {
                    delete _pair_plat[c][i*_nodes+j];
                    delete _pair_nlat[c][i*_nodes+j];
                    delete _pair_flat[c][i*_nodes+j];
                }
            }
        }
    }
  
    if(gWatchOut && (gWatchOut != &cout)) delete gWatchOut;
    if(_stats_out && (_stats_out != &cout)) delete _stats_out;

#ifdef TRACK_FLOWS
    if(_injected_flits_out) delete _injected_flits_out;
    if(_received_flits_out) delete _received_flits_out;
    if(_stored_flits_out) delete _stored_flits_out;
    if(_sent_flits_out) delete _sent_flits_out;
    if(_outstanding_credits_out) delete _outstanding_credits_out;
    if(_ejected_flits_out) delete _ejected_flits_out;
    if(_active_packets_out) delete _active_packets_out;
#endif

#ifdef TRACK_CREDITS
    if(_used_credits_out) delete _used_credits_out;
    if(_free_credits_out) delete _free_credits_out;
    if(_max_credits_out) delete _max_credits_out;
#endif

    PacketReplyInfo::FreeAll();
    Flit::FreeAll();
    Credit::FreeAll();
}


void TrafficManager::_RetireFlit( Flit *f, int dest )
{
    _deadlock_timer = 0;

    assert(_total_in_flight_flits[f->cl].count(f->id) > 0);
    _total_in_flight_flits[f->cl].erase(f->id);
  
    if(f->record) {
        assert(_measured_in_flight_flits[f->cl].count(f->id) > 0);
        _measured_in_flight_flits[f->cl].erase(f->id);
    }

    if ( f->watch ) { 
        *gWatchOut << GetSimTime() << " | "
                   << "node" << dest << " | "
                   << "Retiring flit " << f->id 
                   << " (packet " << f->pid
                   << ", src = " << f->src 
                   << ", dest = " << f->dest
                   << ", hops = " << f->hops
                   << ", flat = " << f->atime - f->itime
                   << ")." << endl;
    }

    if ( f->head && ( f->dest != dest ) ) {
        ostringstream err;
        err << "Flit " << f->id << " arrived at incorrect output " << dest;
        Error( err.str( ) );
    }
  
    if((_slowest_flit[f->cl] < 0) ||
       (_flat_stats[f->cl]->Max() < (f->atime - f->itime)))
        _slowest_flit[f->cl] = f->id;
    _flat_stats[f->cl]->AddSample( f->atime - f->itime);
    if(_pair_stats){
        _pair_flat[f->cl][f->src*_nodes+dest]->AddSample( f->atime - f->itime );
    }
      
    if ( f->tail ) {
        Flit * head;
        if(f->head) {
            head = f;
        } else {
            map<int, Flit *>::iterator iter = _retired_packets[f->cl].find(f->pid);
            assert(iter != _retired_packets[f->cl].end());
            head = iter->second;
            _retired_packets[f->cl].erase(iter);
            assert(head->head);
            assert(f->pid == head->pid);
        }
        if ( f->watch ) { 
            *gWatchOut << GetSimTime() << " | "
                       << "node" << dest << " | "
                       << "Retiring packet " << f->pid 
                       << " (plat = " << f->atime - head->ctime
                       << ", nlat = " << f->atime - head->itime
                       << ", frag = " << (f->atime - head->atime) - (f->id - head->id) // NB: In the spirit of solving problems using ugly hacks, we compute the packet length by taking advantage of the fact that the IDs of flits within a packet are contiguous.
                       << ", src = " << head->src 
                       << ", dest = " << head->dest
                       << ")." << endl;
        }

        //code the source of request, look carefully, its tricky ;)
        if (f->type == Flit::READ_REQUEST || f->type == Flit::WRITE_REQUEST) {
            PacketReplyInfo* rinfo = PacketReplyInfo::New();
            rinfo->source = f->src;
            rinfo->time = f->atime;
            rinfo->record = f->record;
            rinfo->type = f->type;
            _repliesPending[dest].push_back(rinfo);
        } else {
            if(f->type == Flit::READ_REPLY || f->type == Flit::WRITE_REPLY  ){
                _requestsOutstanding[dest]--;
            } else if(f->type == Flit::ANY_TYPE) {
                _requestsOutstanding[f->src]--;
            }
      
        }

        // Only record statistics once per packet (at tail)
        // and based on the simulation state
        if ( ( _sim_state == warming_up ) || f->record ) {
      
            _hop_stats[f->cl]->AddSample( f->hops );

            if((_slowest_packet[f->cl] < 0) ||
               (_plat_stats[f->cl]->Max() < (f->atime - head->itime)))
                _slowest_packet[f->cl] = f->pid;
            _plat_stats[f->cl]->AddSample( f->atime - head->ctime);
            _nlat_stats[f->cl]->AddSample( f->atime - head->itime);
            _frag_stats[f->cl]->AddSample( (f->atime - head->atime) - (f->id - head->id) );
   
            if(_pair_stats){
                _pair_plat[f->cl][f->src*_nodes+dest]->AddSample( f->atime - head->ctime );
                _pair_nlat[f->cl][f->src*_nodes+dest]->AddSample( f->atime - head->itime );
            }
        }
    
        if(f != head) {
            head->Free();
        }
    
    }
  
    if(f->head && !f->tail) {
        _retired_packets[f->cl].insert(make_pair(f->pid, f));
    } else {
        f->Free();
    }

    // ----------------------------------------------------------------
    // Cross-Propagation All-Reduce: hierarchical 4-phase barrier
    // ----------------------------------------------------------------
    // All barrier logic fires on tail flits only (one event per packet).
    if ( _cp_enabled && f->tail ) {

        // Verbose per-packet trace
        if ( _cp_trace ) {
            const char *phase_tag =
                (_cp_phase == 0 && gMeshNet->IsCrossNode(dest) && !gMeshNet->IsCenterNode(dest)) ? "Phase1" :
                (_cp_phase == 0 && dest == _cp_center)                                            ? "Phase2" :
                (_cp_phase == 1 && gMeshNet->IsCrossNode(dest))                                   ? "Phase3" :
                (_cp_phase == 1)                                                                   ? "Phase4" : "?";
            cout << "[CP TRACE][Cycle " << _time << "] ARRIVE "
                 << phase_tag << "  N" << f->src << " -> N" << dest
                 << (f->is_px ? "  Px" : "  Py") << endl;
        }

        if ( _cp_phase == 0 ) {

            // --------------------------------------------------
            // Phase 1: outer → cross  (cross node collects from all neighbors)
            // --------------------------------------------------
            if ( gMeshNet->IsCrossNode(dest) && !gMeshNet->IsCenterNode(dest) ) {
                const int idx = _CpCrossIndex(dest);
                ++_cp_cross_recv[idx];

                if ( _cp_cross_recv[idx] == 4 && !_cp_cross_sent[idx] ) {
                    _cp_cross_sent[idx] = true;
                    ++_cp_cross_all_sent_count;
                    cout << "[CP] Phase1: N" << f->src << " -> N" << dest
                         << "(cross) all arrived"
                         << "  (" << _cp_cross_all_sent_count << "/8)"
                         << "  cycle=" << _time << endl;
                    if ( _cp_cross_all_sent_count == 8 ) {
                        _cp_phase1_local_end_cycle = _time;
                        cout << "[CP] Phase1 DONE: all 8 cross nodes collected"
                             << "  cycle=" << _time << endl;
                    }
                    _InjectCrossToCenter(dest);
                }
            }

            // --------------------------------------------------
            // Phase 2: cross → center  (center collects from all cross nodes)
            // --------------------------------------------------
            if ( dest == _cp_center ) {
                ++_cp_center_recv;
                cout << "[CP] Phase2: N" << f->src << "(cross) -> N" << dest
                     << "(center) arrived"
                     << "  (" << _cp_center_recv << "/8)"
                     << "  cycle=" << _time << endl;

                if ( _cp_center_recv == 8 ) {
                    _cp_phase2_end_cycle = _time;
                    const int reduce_lat = _cp_phase2_end_cycle - _cp_phase1_start_cycle;
                    cout << "[CP] Phase2 DONE: all 8 cross nodes arrived at center"
                         << "  cycle=" << _cp_phase2_end_cycle
                         << "  reduce_latency=" << reduce_lat << " cycles" << endl;
                    _cp_phase = 1;
                    _InjectCenterToCross();
                }
            }

        } else if ( _cp_phase == 1 ) {

            // --------------------------------------------------
            // Phase 3: center → cross  (each cross node receives broadcast)
            // --------------------------------------------------
            if ( gMeshNet->IsCrossNode(dest) && !gMeshNet->IsCenterNode(dest)
                 && f->src == _cp_center ) {
                const int idx = _CpCrossIndex(dest);
                _cp_cross_p2_recv[idx] = true;
                ++_cp_cross_p2_recv_count;
                cout << "[CP] Phase3: N" << f->src << "(center) -> N" << dest
                     << "(cross) arrived"
                     << "  (" << _cp_cross_p2_recv_count << "/8)"
                     << "  cycle=" << _time << endl;
                if ( _cp_cross_p2_recv_count == 8 ) {
                    _cp_phase3_end_cycle = _time;
                    cout << "[CP] Phase3 DONE: all 8 cross nodes received from center"
                         << "  cycle=" << _time << endl;
                }

                if ( _CpIsColCross(dest) ) {
                    _InjectColCrossToOuter(dest);   // Phase 4a: D/2 Px horizontal
                } else {
                    _InjectRowCrossToOuter(dest);   // Phase 4b: D/2 Py vertical
                }
            }

            // --------------------------------------------------
            // Phase 4: cross → outer  (32 outer nodes receive final result)
            // --------------------------------------------------
            if ( !gMeshNet->IsCrossNode(dest) ) {
                ++_cp_outer_recv;

                cout << "[CP] Phase4: N" << f->src << "(cross) -> N" << dest
                     << "(outer) arrived"
                     << "  (" << _cp_outer_recv << "/32)"
                     << "  cycle=" << _time << endl;

                if ( _cp_outer_recv == 32 ) {
                    _cp_phase4_end_cycle = _time;
                    _cp_phase = 2;

                    const int p1_dur = _cp_phase1_local_end_cycle - _cp_phase1_start_cycle;
                    const int p3_dur = (_cp_phase3_end_cycle >= 0)
                                       ? (_cp_phase3_end_cycle - _cp_phase2_end_cycle) : -1;
                    const int p4_dur = _cp_phase4_end_cycle - _cp_phase2_end_cycle;
                    const int reduce_lat    = _cp_phase2_end_cycle - _cp_phase1_start_cycle;
                    const int broadcast_lat = _cp_phase4_end_cycle - _cp_phase2_end_cycle;
                    const int total_lat     = _cp_phase4_end_cycle - _cp_phase1_start_cycle;
                    const int pkt_size      = _GetNextPacketSize(0);

                    cout << "\n====== Cross-Propagation All-Reduce Summary ======\n"
                         << "  Mesh size   : " << _cp_k << "x" << _cp_k
                         << " (" << _nodes << " nodes)\n"
                         << "  Packet size : D=" << pkt_size
                         << " flits  (Px/Py=" << pkt_size/2 << " flits)\n"
                         << "  ---- Per-Phase Timing ----\n"
                         << "  Phase 1 (Outer->Cross)  : " << p1_dur
                         << " cycles  [cycle " << _cp_phase1_start_cycle
                         << " ~ " << _cp_phase1_local_end_cycle << "]\n"
                         << "  Phase 2 (Cross->Center) : "
                         << (_cp_phase2_end_cycle - _cp_phase1_local_end_cycle)
                         << " cycles  [cycle " << _cp_phase1_local_end_cycle
                         << " ~ " << _cp_phase2_end_cycle << "]\n"
                         << "  Phase 3 (Center->Cross) : " << p3_dur
                         << " cycles  [cycle " << _cp_phase2_end_cycle
                         << " ~ " << _cp_phase3_end_cycle << "]\n"
                         << "  Phase 4 (Cross->Outer)  : " << p4_dur
                         << " cycles  [cycle " << _cp_phase2_end_cycle
                         << " ~ " << _cp_phase4_end_cycle
                         << "]  (pipelined with Phase 3)\n"
                         << "  ---- End-to-End ----\n"
                         << "  Reduce    (Phase 1+2)   : " << reduce_lat
                         << " cycles  [outer->cross->center]\n"
                         << "  Broadcast (Phase 3+4)   : " << broadcast_lat
                         << " cycles  [center->cross->outer]\n"
                         << "  Total All-Reduce        : " << total_lat
                         << " cycles\n"
                         << "  ---- Traffic ----\n"
                         << "  Total flits injected    : " << _cp_total_flits
                         << " flits\n"
                         << "==================================================\n"
                         << endl;
                }
            }
        }
    }
}

int TrafficManager::_IssuePacket( int source, int cl )
{
    int result = 0;
    if(_use_read_write[cl]){ //use read and write
        //check queue for waiting replies.
        //check to make sure it is on time yet
        if (!_repliesPending[source].empty()) {
            if(_repliesPending[source].front()->time <= _time) {
                result = -1;
            }
        } else {
      
            //produce a packet
            if(_injection_process[cl]->test(source)) {
	
                //coin toss to determine request type.
                result = (RandomFloat() < _write_fraction[cl]) ? 2 : 1;
	
                _requestsOutstanding[source]++;
            }
        }
    } else { //normal mode
        result = _injection_process[cl]->test(source) ? 1 : 0;
        _requestsOutstanding[source]++;
    } 
    if(result != 0) {
        _packet_seq_no[source]++;
    }
    return result;
}

void TrafficManager::_GeneratePacket( int source, int stype, 
                                      int cl, int time )
{
    assert(stype!=0);

    Flit::FlitType packet_type = Flit::ANY_TYPE;
    int size = _GetNextPacketSize(cl); //input size
    int pid = _cur_pid++;
    assert(_cur_pid);
    int packet_destination = _traffic_pattern[cl]->dest(source);
    bool record = false;
    bool watch = gWatchOut && (_packets_to_watch.count(pid) > 0);
    if(_use_read_write[cl]){
        if(stype > 0) {
            if (stype == 1) {
                packet_type = Flit::READ_REQUEST;
                size = _read_request_size[cl];
            } else if (stype == 2) {
                packet_type = Flit::WRITE_REQUEST;
                size = _write_request_size[cl];
            } else {
                ostringstream err;
                err << "Invalid packet type: " << packet_type;
                Error( err.str( ) );
            }
        } else {
            PacketReplyInfo* rinfo = _repliesPending[source].front();
            if (rinfo->type == Flit::READ_REQUEST) {//read reply
                size = _read_reply_size[cl];
                packet_type = Flit::READ_REPLY;
            } else if(rinfo->type == Flit::WRITE_REQUEST) {  //write reply
                size = _write_reply_size[cl];
                packet_type = Flit::WRITE_REPLY;
            } else {
                ostringstream err;
                err << "Invalid packet type: " << rinfo->type;
                Error( err.str( ) );
            }
            packet_destination = rinfo->source;
            time = rinfo->time;
            record = rinfo->record;
            _repliesPending[source].pop_front();
            rinfo->Free();
        }
    }

    if ((packet_destination <0) || (packet_destination >= _nodes)) {
        ostringstream err;
        err << "Incorrect packet destination " << packet_destination
            << " for stype " << packet_type;
        Error( err.str( ) );
    }

    if ( ( _sim_state == running ) ||
         ( ( _sim_state == draining ) && ( time < _drain_time ) ) ) {
        record = _measure_stats[cl];
    }

    int subnetwork = ((packet_type == Flit::ANY_TYPE) ? 
                      RandomInt(_subnets-1) :
                      _subnet[packet_type]);
  
    if ( watch ) { 
        *gWatchOut << GetSimTime() << " | "
                   << "node" << source << " | "
                   << "Enqueuing packet " << pid
                   << " at time " << time
                   << "." << endl;
    }
  
    for ( int i = 0; i < size; ++i ) {
        Flit * f  = Flit::New();
        f->id     = _cur_id++;
        assert(_cur_id);
        f->pid    = pid;
        f->watch  = watch | (gWatchOut && (_flits_to_watch.count(f->id) > 0));
        f->subnetwork = subnetwork;
        f->src    = source;
        f->ctime  = time;
        f->record = record;
        f->cl     = cl;

        _total_in_flight_flits[f->cl].insert(make_pair(f->id, f));
        if(record) {
            _measured_in_flight_flits[f->cl].insert(make_pair(f->id, f));
        }
    
        if(gTrace){
            cout<<"New Flit "<<f->src<<endl;
        }
        f->type = packet_type;

        if ( i == 0 ) { // Head flit
            f->head = true;
            //packets are only generated to nodes smaller or equal to limit
            f->dest = packet_destination;
        } else {
            f->head = false;
            f->dest = -1;
        }
        switch( _pri_type ) {
        case class_based:
            f->pri = _class_priority[cl];
            assert(f->pri >= 0);
            break;
        case age_based:
            f->pri = numeric_limits<int>::max() - time;
            assert(f->pri >= 0);
            break;
        case sequence_based:
            f->pri = numeric_limits<int>::max() - _packet_seq_no[source];
            assert(f->pri >= 0);
            break;
        default:
            f->pri = 0;
        }
        if ( i == ( size - 1 ) ) { // Tail flit
            f->tail = true;
        } else {
            f->tail = false;
        }
    
        f->vc  = -1;

        if ( f->watch ) { 
            *gWatchOut << GetSimTime() << " | "
                       << "node" << source << " | "
                       << "Enqueuing flit " << f->id
                       << " (packet " << f->pid
                       << ") at time " << time
                       << "." << endl;
        }

        _partial_packets[source][0][cl].push_back( f );
    }
}

// ============================================================
// Cross-Propagation All-Reduce — Node classification helpers
// ============================================================

// col-cross: on center column (col == k/2), but NOT center row (row != k/2)
bool TrafficManager::_CpIsColCross( int node ) const {
    return (node % _cp_k == _cp_k / 2) && (node / _cp_k != _cp_k / 2);
}

// row-cross: on center row (row == k/2), but NOT center column (col != k/2)
bool TrafficManager::_CpIsRowCross( int node ) const {
    return (node / _cp_k == _cp_k / 2) && (node % _cp_k != _cp_k / 2);
}

// Returns 0-7 index for a cross (non-center) node.
// col-cross rows {0,1,3,4} → indices {0,1,2,3}  (skipping center row)
// row-cross cols {0,1,3,4} → indices {4,5,6,7}  (skipping center col)
int TrafficManager::_CpCrossIndex( int node ) const {
    const int half = _cp_k / 2;
    if ( _CpIsColCross(node) ) {
        int row = node / _cp_k;
        return (row < half) ? row : row - 1;   // 0,1,3,4 → 0,1,2,3
    } else {
        int col = node % _cp_k;
        return 4 + ((col < half) ? col : col - 1);  // 0,1,3,4 → 4,5,6,7
    }
}

// col-cross node ID for a given row  (= row * k + k/2)
int TrafficManager::_CpColCrossForRow( int row ) const {
    return row * _cp_k + _cp_k / 2;
}

// row-cross node ID for a given col  (= (k/2)*k + col)
int TrafficManager::_CpRowCrossForCol( int col ) const {
    return (_cp_k / 2) * _cp_k + col;
}

// ============================================================
// Cross-Propagation All-Reduce — Common packet injector
// ============================================================

// Enqueue one packet (src→dest, size flits) into src's partial_packets queue.
// slot selection:
//   is_px=true  → slot 0  (XY routing)
//   is_px=false → slot 1  (YX routing, outer nodes only)
//   center node → slot chosen by dest direction (N/S/E/W)
void TrafficManager::_CpInjectPacket( int src, int dest, int size, int cl, bool is_px )
{
    assert(size > 0);
    const int pid        = _cur_pid++;
    assert(_cur_pid);
    const int subnetwork = (_subnets == 1) ? 0 : RandomInt(_subnets - 1);

    // Slot selection
    int slot;
    if ( src == _cp_center ) {
        // Center node uses directional slots (N=0, S=1, E=2, W=3)
        const int half = _cp_k / 2;
        const int dr   = dest / _cp_k;
        const int dc   = dest % _cp_k;
        if      ( dr < half ) slot = 0;
        else if ( dr > half ) slot = 1;
        else if ( dc > half ) slot = 2;
        else                  slot = 3;
    } else if ( gMeshNet->IsCrossNode(src) ) {
        // Cross nodes (non-center): 2 NI slots (directional).
        // Phase 2 (→ center)  : slot 0.
        // Phase 4 col-cross   : slot 0 = west (dest_col < half), slot 1 = east
        // Phase 4 row-cross   : slot 0 = north (dest_row < half), slot 1 = south
        if ( dest == _cp_center ) {
            slot = 0;
        } else {
            const int half    = _cp_k / 2;
            const int src_col = src % _cp_k;
            if ( src_col == half ) {
                slot = ( dest % _cp_k < half ) ? 0 : 1;   // col-cross: W=0, E=1
            } else {
                slot = ( dest / _cp_k < half ) ? 0 : 1;   // row-cross: N=0, S=1
            }
        }
    } else {
        // Outer nodes: Px → slot 0 (XY), Py → slot 1 (YX)
        slot = is_px ? 0 : 1;
    }

    for ( int i = 0; i < size; ++i ) {
        Flit *f       = Flit::New();
        f->id         = _cur_id++;
        assert(_cur_id);
        f->pid        = pid;
        f->watch      = gWatchOut && (_packets_to_watch.count(pid) > 0);
        f->subnetwork = subnetwork;
        f->src        = src;
        f->ctime      = _time;
        f->record     = false;
        f->cl         = cl;
        f->type       = Flit::ANY_TYPE;
        f->is_px      = is_px;

        _total_in_flight_flits[cl].insert(make_pair(f->id, f));

        f->head = (i == 0);
        f->dest = (i == 0) ? dest : -1;
        f->pri  = 0;
        f->tail = (i == size - 1);
        f->vc   = -1;

        _partial_packets[src][slot][cl].push_back(f);
    }

    _requestsOutstanding[src]++;
    _cp_total_flits += size;

    if ( _cp_trace ) {
        const char *phase_tag =
            (src != _cp_center && !gMeshNet->IsCrossNode(src)) ? "Phase1" :
            (gMeshNet->IsCrossNode(src) && dest == _cp_center) ? "Phase2" :
            (src == _cp_center)                                 ? "Phase3" : "Phase4";
        cout << "[CP TRACE][Cycle " << _time << "] INJECT "
             << phase_tag << "  N" << src << " -> N" << dest
             << (is_px ? "  Px" : "  Py")
             << "  " << size << " flits" << endl;
    }
}

// ============================================================
// Cross-Propagation All-Reduce — Phase 0: Outer → Cross
// ============================================================

// One-shot: outer nodes inject Px → col-cross, Py → row-cross (both D/2 flits).
// Cross nodes and center node do NOT inject here; they wait for incoming packets.
void TrafficManager::_InjectAllPhase1()
{
    assert(gMeshNet != nullptr);
    const Mesh *mesh    = static_cast<const Mesh *>(gMeshNet);
    const int   cl      = 0;
    const int   half    = _GetNextPacketSize(cl) / 2;

    _cp_phase1_start_cycle = _time;

    for ( int node = 0; node < _nodes; ++node ) {
        if ( mesh->IsCrossNode(node) ) continue;   // cross/center handled by barriers

        const int row       = node / _cp_k;
        const int col       = node % _cp_k;
        const int col_cross = _CpColCrossForRow(row);
        const int row_cross = _CpRowCrossForCol(col);

        // Px: XY routing → col-cross of same row
        _CpInjectPacket(node, col_cross, half, cl, /*is_px=*/true);
        // Py: YX routing → row-cross of same col
        _CpInjectPacket(node, row_cross, half, cl, /*is_px=*/false);
    }

    _cp_phase1_injected = true;
}

// ============================================================
// Cross-Propagation All-Reduce — Phase 0.5: Cross → Center
// ============================================================

// Called when cross_node's local barrier fires (received all 4 outer packets).
// Injects one aggregated-sum packet (D/2 flits) from cross_node to center.
void TrafficManager::_InjectCrossToCenter( int cross_node )
{
    const int cl   = 0;
    const int size = _GetNextPacketSize(cl) / 2;

    _CpInjectPacket(cross_node, _cp_center, size, cl, /*is_px=*/true);
}

// ============================================================
// Cross-Propagation All-Reduce — Phase 3: Center → 8 Cross
// ============================================================

// Called when the center global barrier fires (received all 8 cross packets).
// Center injects D/2 flits to each of the 8 cross nodes.
// Col-cross will forward as Px; row-cross will forward as Py.
void TrafficManager::_InjectCenterToCross()
{
    assert(gMeshNet != nullptr);
    const Mesh *mesh = static_cast<const Mesh *>(gMeshNet);
    const int   cl   = 0;
    const int   size = _GetNextPacketSize(cl) / 2;   // D/2 per cross node
    const int   half = _cp_k / 2;

    // Far-first: 2-hop cross nodes (N2/N10/N14/N22) before 1-hop (N7/N11/N13/N17).
    // Each center slot (N/S/E/W) queues the far node first so it departs before
    // the near node, avoiding link contention at the intermediate near-cross router.
    for ( int pass = 2; pass >= 1; --pass ) {
        for ( int node = 0; node < _nodes; ++node ) {
            if ( !mesh->IsCrossNode(node) || mesh->IsCenterNode(node) ) continue;
            const int dist = abs(node / _cp_k - half) + abs(node % _cp_k - half);
            if ( dist != pass ) continue;
            _CpInjectPacket(_cp_center, node, size, cl, /*is_px=*/true);
        }
    }
}

// ============================================================
// Cross-Propagation All-Reduce — Phase 4a: Col-Cross → Outer (Px)
// ============================================================

// Called when col_cross_node receives D/2 from center (Phase 3).
// Broadcasts D/2 Px to each of its 4 outer nodes in the SAME ROW (horizontal).
void TrafficManager::_InjectColCrossToOuter( int col_cross_node )
{
    const int cl   = 0;
    const int size = _GetNextPacketSize(cl) / 2;   // D/2
    const int row  = col_cross_node / _cp_k;
    const int half = _cp_k / 2;

    // Far-first within each slot:
    // Slot 0 (west): col 0 (2-hop) → col half-1 (1-hop)  — ascending
    for ( int col = 0; col < half; ++col )
        _CpInjectPacket(col_cross_node, row * _cp_k + col, size, cl, /*is_px=*/true);

    // Slot 1 (east): col k-1 (2-hop) → col half+1 (1-hop) — descending
    for ( int col = _cp_k - 1; col > half; --col )
        _CpInjectPacket(col_cross_node, row * _cp_k + col, size, cl, /*is_px=*/true);
}

// ============================================================
// Cross-Propagation All-Reduce — Phase 4b: Row-Cross → Outer (Py)
// ============================================================

// Called when row_cross_node receives D/2 from center (Phase 3).
// Broadcasts D/2 Py to each of its 4 outer nodes in the SAME COLUMN (vertical).
void TrafficManager::_InjectRowCrossToOuter( int row_cross_node )
{
    const int cl   = 0;
    const int size = _GetNextPacketSize(cl) / 2;   // D/2
    const int col  = row_cross_node % _cp_k;
    const int half = _cp_k / 2;

    // Far-first within each slot:
    // Slot 0 (north): row 0 (2-hop) → row half-1 (1-hop) — ascending
    for ( int row = 0; row < half; ++row )
        _CpInjectPacket(row_cross_node, row * _cp_k + col, size, cl, /*is_px=*/false);

    // Slot 1 (south): row k-1 (2-hop) → row half+1 (1-hop) — descending
    for ( int row = _cp_k - 1; row > half; --row )
        _CpInjectPacket(row_cross_node, row * _cp_k + col, size, cl, /*is_px=*/false);
}

// ============================================================

void TrafficManager::_Inject(){

    // Cross-Propagation: replace the normal per-cycle injection with a
    // single one-shot bulk injection for Phase 1.  After that, Phase 2
    // packets are enqueued directly by _TriggerPhase2(), so there is
    // nothing for _Inject() to do on subsequent cycles.
    if(_cp_enabled) {
        if(!_cp_phase1_injected) {
            _InjectAllPhase1();
        }
        return;
    }

    for ( int input = 0; input < _nodes; ++input ) {
        for ( int c = 0; c < _classes; ++c ) {
            // Potentially generate packets for any (input,class)
            // that is currently empty (normal mode always uses slot 0)
            if ( _partial_packets[input][0][c].empty() ) {
                bool generated = false;
                while( !generated && ( _qtime[input][c] <= _time ) ) {
                    int stype = _IssuePacket( input, c );
	  
                    if ( stype != 0 ) { //generate a packet
                        _GeneratePacket( input, stype, c, 
                                         _include_queuing==1 ? 
                                         _qtime[input][c] : _time );
                        generated = true;
                    }
                    // only advance time if this is not a reply packet
                    if(!_use_read_write[c] || (stype >= 0)){
                        ++_qtime[input][c];
                    }
                }
	
                if ( ( _sim_state == draining ) && 
                     ( _qtime[input][c] > _drain_time ) ) {
                    _qdrained[input][c] = true;
                }
            }
        }
    }
}

void TrafficManager::_Step( )
{
    bool flits_in_flight = false;
    for(int c = 0; c < _classes; ++c) {
        flits_in_flight |= !_total_in_flight_flits[c].empty();
    }
    if(flits_in_flight && (_deadlock_timer++ >= _deadlock_warn_timeout)){
        _deadlock_timer = 0;
        cout << "WARNING: Possible network deadlock.\n";
    }

    // flits[subnet][node] holds all ejected flits for that node this cycle.
    // Center node can eject up to 4 flits/cycle (one per directional slot).
    // flits[subnet][node] stores (flit, ejection_slot) pairs so the
    // retirement loop can write credits to the correct slot credit channel.
    vector<map<int, vector<pair<Flit *, int>>>> flits(_subnets);

    for ( int subnet = 0; subnet < _subnets; ++subnet ) {
        for ( int n = 0; n < _nodes; ++n ) {
            // Read ejection slot 0 via standard Network::ReadFlit.
            Flit * const f = _net[subnet]->ReadFlit( n );
            if ( f ) {
                if(f->watch) {
                    *gWatchOut << GetSimTime() << " | "
                               << "node" << n << " | "
                               << "Ejecting flit " << f->id
                               << " (packet " << f->pid << ")"
                               << " from VC " << f->vc
                               << "." << endl;
                }
                flits[subnet][n].push_back({f, 0});
                if((_sim_state == warming_up) || (_sim_state == running)) {
                    ++_accepted_flits[f->cl][n];
                    if(f->tail) {
                        ++_accepted_packets[f->cl][n];
                    }
                }
            }

            // Read extra ejection slots (center node has 4 total).
            {
                const Mesh *emesh = dynamic_cast<const Mesh *>(_net[subnet]);
                int ne = emesh ? emesh->NumEjectionSlots(n) : 1;
                for ( int eslot = 1; eslot < ne; ++eslot ) {
                    Flit * const ef = emesh->ReadFlitFromEjectSlot(n, eslot);
                    if ( ef ) {
                        if(ef->watch) {
                            *gWatchOut << GetSimTime() << " | "
                                       << "node" << n << " | "
                                       << "Ejecting flit " << ef->id
                                       << " (packet " << ef->pid << ")"
                                       << " from VC " << ef->vc
                                       << " slot " << eslot << "." << endl;
                        }
                        flits[subnet][n].push_back({ef, eslot});
                        if((_sim_state == warming_up) || (_sim_state == running)) {
                            ++_accepted_flits[ef->cl][n];
                            if(ef->tail) {
                                ++_accepted_packets[ef->cl][n];
                            }
                        }
                    }
                }
            }

            {
                const Mesh * cmesh = dynamic_cast<const Mesh *>(_net[subnet]);
                int ni = cmesh ? cmesh->NumInjectionSlots(n) : 1;
                for ( int slot = 0; slot < ni; ++slot ) {
                    Credit * const c = cmesh
                        ? cmesh->ReadCreditFromSlot(n, slot)
                        : _net[subnet]->ReadCredit(n);
                    if ( c ) {
#ifdef TRACK_FLOWS
                        for(set<int>::const_iterator iter = c->vc.begin(); iter != c->vc.end(); ++iter) {
                            int const vc = *iter;
                            assert(!_outstanding_classes[n][subnet][vc].empty());
                            int cl = _outstanding_classes[n][subnet][vc].front();
                            _outstanding_classes[n][subnet][vc].pop();
                            assert(_outstanding_credits[cl][subnet][n] > 0);
                            --_outstanding_credits[cl][subnet][n];
                        }
#endif
                        _buf_states[n][slot][subnet]->ProcessCredit(c);
                        c->Free();
                    }
                }
            }
        }
        _net[subnet]->ReadInputs( );
    }
  
    if ( !_empty_network ) {
        _Inject();
    }

    for(int subnet = 0; subnet < _subnets; ++subnet) {

        for(int n = 0; n < _nodes; ++n) {

            const Mesh * mesh_inject = dynamic_cast<const Mesh *>(_net[subnet]);
            int ni = mesh_inject ? mesh_inject->NumInjectionSlots(n) : 1;

            for(int slot = 0; slot < ni; ++slot) {

            Flit * f = NULL;

            BufferState * const dest_buf = _buf_states[n][slot][subnet];

            int const last_class = _last_class[n][slot][subnet];

            int class_limit = _classes;

            if(_hold_switch_for_packet) {
                list<Flit *> const & pp = _partial_packets[n][slot][last_class];
                if(!pp.empty() && !pp.front()->head &&
                   !dest_buf->IsFullFor(pp.front()->vc)) {
                    f = pp.front();
                    assert(f->vc == _last_vc[n][slot][subnet][last_class]);

                    // if we're holding the connection, we don't need to check that class
                    // again in the for loop
                    --class_limit;
                }
            }

            for(int i = 1; i <= class_limit; ++i) {

                int const c = (last_class + i) % _classes;

                list<Flit *> const & pp = _partial_packets[n][slot][c];

                if(pp.empty()) {
                    continue;
                }

                Flit * const cf = pp.front();
                assert(cf);
                assert(cf->cl == c);

                if(cf->subnetwork != subnet) {
                    continue;
                }

                if(f && (f->pri >= cf->pri)) {
                    continue;
                }

                if(cf->head && cf->vc == -1) { // Find first available VC

                    OutputSet route_set;
                    _rf(NULL, cf, -1, &route_set, true);
                    set<OutputSet::sSetElement> const & os = route_set.GetSet();
                    assert(os.size() == 1);
                    OutputSet::sSetElement const & se = *os.begin();
                    assert(se.output_port == -1);
                    int vc_start = se.vc_start;
                    int vc_end = se.vc_end;
                    int vc_count = vc_end - vc_start + 1;
                    if(_noq) {
                        assert(_lookahead_routing);
                        const FlitChannel * inject = _net[subnet]->GetInject(n);
                        const Router * router = inject->GetSink();
                        assert(router);
                        int in_channel = inject->GetSinkPort();

                        // NOTE: Because the lookahead is not for injection, but for the
                        // first hop, we have to temporarily set cf's VC to be non-negative
                        // in order to avoid seting of an assertion in the routing function.
                        cf->vc = vc_start;
                        _rf(router, cf, in_channel, &cf->la_route_set, false);
                        cf->vc = -1;

                        if(cf->watch) {
                            *gWatchOut << GetSimTime() << " | "
                                       << "node" << n << " | "
                                       << "Generating lookahead routing info for flit " << cf->id
                                       << " (NOQ)." << endl;
                        }
                        set<OutputSet::sSetElement> const sl = cf->la_route_set.GetSet();
                        assert(sl.size() == 1);
                        int next_output = sl.begin()->output_port;
                        vc_count /= router->NumOutputs();
                        vc_start += next_output * vc_count;
                        vc_end = vc_start + vc_count - 1;
                        assert(vc_start >= se.vc_start && vc_start <= se.vc_end);
                        assert(vc_end >= se.vc_start && vc_end <= se.vc_end);
                        assert(vc_start <= vc_end);
                    }
                    if(cf->watch) {
                        *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                                   << "Finding output VC for flit " << cf->id
                                   << ":" << endl;
                    }
                    for(int i = 1; i <= vc_count; ++i) {
                        int const lvc = _last_vc[n][slot][subnet][c];
                        int const vc =
                            (lvc < vc_start || lvc > vc_end) ?
                            vc_start :
                            (vc_start + (lvc - vc_start + i) % vc_count);
                        assert((vc >= vc_start) && (vc <= vc_end));
                        if(!dest_buf->IsAvailableFor(vc)) {
                            if(cf->watch) {
                                *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                                           << "  Output VC " << vc << " is busy." << endl;
                            }
                        } else {
                            if(dest_buf->IsFullFor(vc)) {
                                if(cf->watch) {
                                    *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                                               << "  Output VC " << vc << " is full." << endl;
                                }
                            } else {
                                if(cf->watch) {
                                    *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                                               << "  Selected output VC " << vc << "." << endl;
                                }
                                cf->vc = vc;
                                break;
                            }
                        }
                    }
                }

                if(cf->vc == -1) {
                    if(cf->watch) {
                        *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                                   << "No output VC found for flit " << cf->id
                                   << "." << endl;
                    }
                } else {
                    if(dest_buf->IsFullFor(cf->vc)) {
                        if(cf->watch) {
                            *gWatchOut << GetSimTime() << " | " << FullName() << " | "
                                       << "Selected output VC " << cf->vc
                                       << " is full for flit " << cf->id
                                       << "." << endl;
                        }
                    } else {
                        f = cf;
                    }
                }
            }

            if(f) {

                assert(f->subnetwork == subnet);

                int const c = f->cl;

                if(f->head) {

                    if (_lookahead_routing) {
                        if(!_noq) {
                            const FlitChannel * inject = _net[subnet]->GetInject(n);
                            const Router * router = inject->GetSink();
                            assert(router);
                            int in_channel = inject->GetSinkPort();
                            _rf(router, f, in_channel, &f->la_route_set, false);
                            if(f->watch) {
                                *gWatchOut << GetSimTime() << " | "
                                           << "node" << n << " | "
                                           << "Generating lookahead routing info for flit " << f->id
                                           << "." << endl;
                            }
                        } else if(f->watch) {
                            *gWatchOut << GetSimTime() << " | "
                                       << "node" << n << " | "
                                       << "Already generated lookahead routing info for flit " << f->id
                                       << " (NOQ)." << endl;
                        }
                    } else {
                        f->la_route_set.Clear();
                    }

                    dest_buf->TakeBuffer(f->vc);
                    _last_vc[n][slot][subnet][c] = f->vc;
                }

                _last_class[n][slot][subnet] = c;

                _partial_packets[n][slot][c].pop_front();

#ifdef TRACK_FLOWS
                ++_outstanding_credits[c][subnet][n];
                _outstanding_classes[n][subnet][f->vc].push(c);
#endif

                dest_buf->SendingFlit(f);

                if(_pri_type == network_age_based) {
                    f->pri = numeric_limits<int>::max() - _time;
                    assert(f->pri >= 0);
                }

                if(f->watch) {
                    *gWatchOut << GetSimTime() << " | "
                               << "node" << n << " | "
                               << "Injecting flit " << f->id
                               << " into subnet " << subnet
                               << " at time " << _time
                               << " with priority " << f->pri
                               << "." << endl;
                }
                f->itime = _time;

                // Pass VC "back"
                if(!_partial_packets[n][slot][c].empty() && !f->tail) {
                    Flit * const nf = _partial_packets[n][slot][c].front();
                    nf->vc = f->vc;
                }

                if((_sim_state == warming_up) || (_sim_state == running)) {
                    ++_sent_flits[c][n];
                    if(f->head) {
                        ++_sent_packets[c][n];
                    }
                }

#ifdef TRACK_FLOWS
                ++_injected_flits[c][n];
#endif

                _net[subnet]->WriteFlit(f, n);

            }

            } // for slot
        }
    }

    for(int subnet = 0; subnet < _subnets; ++subnet) {
        for(int n = 0; n < _nodes; ++n) {
            auto iter = flits[subnet].find(n);
            if(iter != flits[subnet].end()) {
                Mesh *emesh = dynamic_cast<Mesh *>(_net[subnet]);
                for ( auto & fp : iter->second ) {
                    Flit * f    = fp.first;
                    int    eslot = fp.second;
                    f->atime = _time;
                    if(f->watch) {
                        *gWatchOut << GetSimTime() << " | "
                                   << "node" << n << " | "
                                   << "Injecting credit for VC " << f->vc
                                   << " into subnet " << subnet
                                   << "." << endl;
                    }
                    Credit * const c = Credit::New();
                    c->vc.insert(f->vc);
                    if (emesh) {
                        emesh->WriteEjectCredit(c, n, eslot);
                    } else {
                        _net[subnet]->WriteCredit(c, n);
                    }

#ifdef TRACK_FLOWS
                    ++_ejected_flits[f->cl][n];
#endif

                    _RetireFlit(f, n);
                }
            }
        }
        flits[subnet].clear();
        _net[subnet]->Evaluate( );
        _net[subnet]->WriteOutputs( );
    }

    ++_time;
    assert(_time);
    if(gTrace){
        cout<<"TIME "<<_time<<endl;
    }

}
  
bool TrafficManager::_PacketsOutstanding( ) const
{
    for ( int c = 0; c < _classes; ++c ) {
        if ( _measure_stats[c] ) {
            if ( _measured_in_flight_flits[c].empty() ) {
	
                for ( int s = 0; s < _nodes; ++s ) {
                    if ( !_qdrained[s][c] ) {
#ifdef DEBUG_DRAIN
                        cout << "waiting on queue " << s << " class " << c;
                        cout << ", time = " << _time << " qtime = " << _qtime[s][c] << endl;
#endif
                        return true;
                    }
                }
            } else {
#ifdef DEBUG_DRAIN
                cout << "in flight = " << _measured_in_flight_flits[c].size() << endl;
#endif
                return true;
            }
        }
    }
    return false;
}

void TrafficManager::_ClearStats( )
{
    _slowest_flit.assign(_classes, -1);
    _slowest_packet.assign(_classes, -1);

    for ( int c = 0; c < _classes; ++c ) {

        _plat_stats[c]->Clear( );
        _nlat_stats[c]->Clear( );
        _flat_stats[c]->Clear( );

        _frag_stats[c]->Clear( );

        _sent_packets[c].assign(_nodes, 0);
        _accepted_packets[c].assign(_nodes, 0);
        _sent_flits[c].assign(_nodes, 0);
        _accepted_flits[c].assign(_nodes, 0);

#ifdef TRACK_STALLS
        _buffer_busy_stalls[c].assign(_subnets*_routers, 0);
        _buffer_conflict_stalls[c].assign(_subnets*_routers, 0);
        _buffer_full_stalls[c].assign(_subnets*_routers, 0);
        _buffer_reserved_stalls[c].assign(_subnets*_routers, 0);
        _crossbar_conflict_stalls[c].assign(_subnets*_routers, 0);
#endif
        if(_pair_stats){
            for ( int i = 0; i < _nodes; ++i ) {
                for ( int j = 0; j < _nodes; ++j ) {
                    _pair_plat[c][i*_nodes+j]->Clear( );
                    _pair_nlat[c][i*_nodes+j]->Clear( );
                    _pair_flat[c][i*_nodes+j]->Clear( );
                }
            }
        }
        _hop_stats[c]->Clear();

    }

    _reset_time = _time;
}

void TrafficManager::_ComputeStats( const vector<int> & stats, int *sum, int *min, int *max, int *min_pos, int *max_pos ) const 
{
    int const count = stats.size();
    assert(count > 0);

    if(min_pos) {
        *min_pos = 0;
    }
    if(max_pos) {
        *max_pos = 0;
    }

    if(min) {
        *min = stats[0];
    }
    if(max) {
        *max = stats[0];
    }

    *sum = stats[0];

    for ( int i = 1; i < count; ++i ) {
        int curr = stats[i];
        if ( min  && ( curr < *min ) ) {
            *min = curr;
            if ( min_pos ) {
                *min_pos = i;
            }
        }
        if ( max && ( curr > *max ) ) {
            *max = curr;
            if ( max_pos ) {
                *max_pos = i;
            }
        }
        *sum += curr;
    }
}

void TrafficManager::_DisplayRemaining( ostream & os ) const 
{
    for(int c = 0; c < _classes; ++c) {

        map<int, Flit *>::const_iterator iter;
        int i;

        os << "Class " << c << ":" << endl;

        os << "Remaining flits: ";
        for ( iter = _total_in_flight_flits[c].begin( ), i = 0;
              ( iter != _total_in_flight_flits[c].end( ) ) && ( i < 10 );
              iter++, i++ ) {
            os << iter->first << " ";
        }
        if(_total_in_flight_flits[c].size() > 10)
            os << "[...] ";
    
        os << "(" << _total_in_flight_flits[c].size() << " flits)" << endl;
    
        os << "Measured flits: ";
        for ( iter = _measured_in_flight_flits[c].begin( ), i = 0;
              ( iter != _measured_in_flight_flits[c].end( ) ) && ( i < 10 );
              iter++, i++ ) {
            os << iter->first << " ";
        }
        if(_measured_in_flight_flits[c].size() > 10)
            os << "[...] ";
    
        os << "(" << _measured_in_flight_flits[c].size() << " flits)" << endl;
    
    }
}

bool TrafficManager::_SingleSim( )
{
    int converged = 0;
  
    //once warmed up, we require 3 converging runs to end the simulation 
    vector<double> prev_latency(_classes, 0.0);
    vector<double> prev_accepted(_classes, 0.0);
    bool clear_last = false;
    int total_phases = 0;
    while( ( total_phases < _max_samples ) &&
           ( ( _sim_state != running ) ||
             ( converged < 3 ) ) ) {

        if ( clear_last || (( ( _sim_state == warming_up ) && ( ( total_phases % 2 ) == 0 ) )) ) {
            clear_last = false;
            _ClearStats( );
        }

        for ( int iter = 0; iter < _sample_period; ++iter ) {
            _Step( );
            // Cross-Propagation: All-Reduce finished — stop immediately after
            // the step in which _RetireFlit printed the completion message.
            if ( _cp_enabled && _cp_phase == 2 ) {
                goto cp_allreduce_done;
            }
        }

        //cout << _sim_state << endl;

        UpdateStats();
        DisplayStats();
    
        int lat_exc_class = -1;
        int lat_chg_exc_class = -1;
        int acc_chg_exc_class = -1;
    
        for(int c = 0; c < _classes; ++c) {
      
            if(_measure_stats[c] == 0) {
                continue;
            }

            double cur_latency = _plat_stats[c]->Average( );

            int total_accepted_count;
            _ComputeStats( _accepted_flits[c], &total_accepted_count );
            double total_accepted_rate = (double)total_accepted_count / (double)(_time - _reset_time);
            double cur_accepted = total_accepted_rate / (double)_nodes;

            double latency_change = fabs((cur_latency - prev_latency[c]) / cur_latency);
            prev_latency[c] = cur_latency;

            double accepted_change = fabs((cur_accepted - prev_accepted[c]) / cur_accepted);
            prev_accepted[c] = cur_accepted;

            double latency = (double)_plat_stats[c]->Sum();
            double count = (double)_plat_stats[c]->NumSamples();
      
            map<int, Flit *>::const_iterator iter;
            for(iter = _total_in_flight_flits[c].begin(); 
                iter != _total_in_flight_flits[c].end(); 
                iter++) {
                latency += (double)(_time - iter->second->ctime);
                count++;
            }
      
            if((lat_exc_class < 0) &&
               (_latency_thres[c] >= 0.0) &&
               ((latency / count) > _latency_thres[c])) {
                lat_exc_class = c;
            }
      
            cout << "latency change    = " << latency_change << endl;
            if(lat_chg_exc_class < 0) {
                if((_sim_state == warming_up) &&
                   (_warmup_threshold[c] >= 0.0) &&
                   (latency_change > _warmup_threshold[c])) {
                    lat_chg_exc_class = c;
                } else if((_sim_state == running) &&
                          (_stopping_threshold[c] >= 0.0) &&
                          (latency_change > _stopping_threshold[c])) {
                    lat_chg_exc_class = c;
                }
            }
      
            cout << "throughput change = " << accepted_change << endl;
            if(acc_chg_exc_class < 0) {
                if((_sim_state == warming_up) &&
                   (_acc_warmup_threshold[c] >= 0.0) &&
                   (accepted_change > _acc_warmup_threshold[c])) {
                    acc_chg_exc_class = c;
                } else if((_sim_state == running) &&
                          (_acc_stopping_threshold[c] >= 0.0) &&
                          (accepted_change > _acc_stopping_threshold[c])) {
                    acc_chg_exc_class = c;
                }
            }
      
        }
    
        // Fail safe for latency mode, throughput will ust continue
        if ( _measure_latency && ( lat_exc_class >= 0 ) ) {
      
            cout << "Average latency for class " << lat_exc_class << " exceeded " << _latency_thres[lat_exc_class] << " cycles. Aborting simulation." << endl;
            converged = 0; 
            _sim_state = draining;
            _drain_time = _time;
            if(_stats_out) {
                WriteStats(*_stats_out);
            }
            break;
      
        }
    
        if ( _sim_state == warming_up ) {
            if ( ( _warmup_periods > 0 ) ? 
                 ( total_phases + 1 >= _warmup_periods ) :
                 ( ( !_measure_latency || ( lat_chg_exc_class < 0 ) ) &&
                   ( acc_chg_exc_class < 0 ) ) ) {
                cout << "Warmed up ..." <<  "Time used is " << _time << " cycles" <<endl;
                clear_last = true;
                _sim_state = running;
            }
        } else if(_sim_state == running) {
            if ( ( !_measure_latency || ( lat_chg_exc_class < 0 ) ) &&
                 ( acc_chg_exc_class < 0 ) ) {
                ++converged;
            } else {
                converged = 0;
            }
        }
        ++total_phases;
    }
  
    cp_allreduce_done:
    // Cross-Propagation early exit: skip all draining and return success.
    if ( _cp_enabled && _cp_phase == 2 ) {
        return true;
    }

    if ( _sim_state == running ) {
        ++converged;

        _sim_state  = draining;
        _drain_time = _time;

        if ( _measure_latency ) {
            cout << "Draining all recorded packets ..." << endl;
            int empty_steps = 0;
            while( _PacketsOutstanding( ) ) { 
                _Step( ); 
	
                ++empty_steps;
	
                if ( empty_steps % 1000 == 0 ) {
	  
                    int lat_exc_class = -1;
	  
                    for(int c = 0; c < _classes; c++) {
	    
                        double threshold = _latency_thres[c];
	    
                        if(threshold < 0.0) {
                            continue;
                        }
	    
                        double acc_latency = _plat_stats[c]->Sum();
                        double acc_count = (double)_plat_stats[c]->NumSamples();
	    
                        map<int, Flit *>::const_iterator iter;
                        for(iter = _total_in_flight_flits[c].begin(); 
                            iter != _total_in_flight_flits[c].end(); 
                            iter++) {
                            acc_latency += (double)(_time - iter->second->ctime);
                            acc_count++;
                        }
	    
                        if((acc_latency / acc_count) > threshold) {
                            lat_exc_class = c;
                            break;
                        }
                    }
	  
                    if(lat_exc_class >= 0) {
                        cout << "Average latency for class " << lat_exc_class << " exceeded " << _latency_thres[lat_exc_class] << " cycles. Aborting simulation." << endl;
                        converged = 0; 
                        _sim_state = warming_up;
                        if(_stats_out) {
                            WriteStats(*_stats_out);
                        }
                        break;
                    }
	  
                    _DisplayRemaining( ); 
	  
                }
            }
        }
    } else {
        cout << "Too many sample periods needed to converge" << endl;
    }
  
    return ( converged > 0 );
}

bool TrafficManager::Run( )
{
    for ( int sim = 0; sim < _total_sims; ++sim ) {

        _time = 0;

        //remove any pending request from the previous simulations
        _requestsOutstanding.assign(_nodes, 0);
        for (int i=0;i<_nodes;i++) {
            while(!_repliesPending[i].empty()) {
                _repliesPending[i].front()->Free();
                _repliesPending[i].pop_front();
            }
        }

        //reset queuetime for all sources
        for ( int s = 0; s < _nodes; ++s ) {
            _qtime[s].assign(_classes, 0);
            _qdrained[s].assign(_classes, false);
        }

        // warm-up ...
        // reset stats, all packets after warmup_time marked
        // converge
        // draing, wait until all packets finish
        _sim_state    = warming_up;
  
        _ClearStats( );

        for(int c = 0; c < _classes; ++c) {
            _traffic_pattern[c]->reset();
            _injection_process[c]->reset();
        }

        if ( !_SingleSim( ) ) {
            cout << "Simulation unstable, ending ..." << endl;
            return false;
        }

        // Empty any remaining packets
        cout << "Draining remaining packets ..." << endl;
        _empty_network = true;
        int empty_steps = 0;

        bool packets_left = false;
        for(int c = 0; c < _classes; ++c) {
            packets_left |= !_total_in_flight_flits[c].empty();
        }

        while( packets_left ) { 
            _Step( ); 

            ++empty_steps;

            if ( empty_steps % 1000 == 0 ) {
                _DisplayRemaining( ); 
            }
      
            packets_left = false;
            for(int c = 0; c < _classes; ++c) {
                packets_left |= !_total_in_flight_flits[c].empty();
            }
        }
        //wait until all the credits are drained as well
        while(Credit::OutStanding()!=0){
            _Step();
        }
        _empty_network = false;

        //for the love of god don't ever say "Time taken" anywhere else
        //the power script depend on it
        cout << "Time taken is " << _time << " cycles" <<endl; 

        if(_stats_out) {
            WriteStats(*_stats_out);
        }
        _UpdateOverallStats();
    }
  
    DisplayOverallStats();
    if(_print_csv_results) {
        DisplayOverallStatsCSV();
    }
  
    return true;
}

void TrafficManager::_UpdateOverallStats() {
    for ( int c = 0; c < _classes; ++c ) {
    
        if(_measure_stats[c] == 0) {
            continue;
        }
    
        _overall_min_plat[c] += _plat_stats[c]->Min();
        _overall_avg_plat[c] += _plat_stats[c]->Average();
        _overall_max_plat[c] += _plat_stats[c]->Max();
        _overall_min_nlat[c] += _nlat_stats[c]->Min();
        _overall_avg_nlat[c] += _nlat_stats[c]->Average();
        _overall_max_nlat[c] += _nlat_stats[c]->Max();
        _overall_min_flat[c] += _flat_stats[c]->Min();
        _overall_avg_flat[c] += _flat_stats[c]->Average();
        _overall_max_flat[c] += _flat_stats[c]->Max();
    
        _overall_min_frag[c] += _frag_stats[c]->Min();
        _overall_avg_frag[c] += _frag_stats[c]->Average();
        _overall_max_frag[c] += _frag_stats[c]->Max();

        _overall_hop_stats[c] += _hop_stats[c]->Average();

        int count_min, count_sum, count_max;
        double rate_min, rate_sum, rate_max;
        double rate_avg;
        double time_delta = (double)(_drain_time - _reset_time);
        _ComputeStats( _sent_flits[c], &count_sum, &count_min, &count_max );
        rate_min = (double)count_min / time_delta;
        rate_sum = (double)count_sum / time_delta;
        rate_max = (double)count_max / time_delta;
        rate_avg = rate_sum / (double)_nodes;
        _overall_min_sent[c] += rate_min;
        _overall_avg_sent[c] += rate_avg;
        _overall_max_sent[c] += rate_max;
        _ComputeStats( _sent_packets[c], &count_sum, &count_min, &count_max );
        rate_min = (double)count_min / time_delta;
        rate_sum = (double)count_sum / time_delta;
        rate_max = (double)count_max / time_delta;
        rate_avg = rate_sum / (double)_nodes;
        _overall_min_sent_packets[c] += rate_min;
        _overall_avg_sent_packets[c] += rate_avg;
        _overall_max_sent_packets[c] += rate_max;
        _ComputeStats( _accepted_flits[c], &count_sum, &count_min, &count_max );
        rate_min = (double)count_min / time_delta;
        rate_sum = (double)count_sum / time_delta;
        rate_max = (double)count_max / time_delta;
        rate_avg = rate_sum / (double)_nodes;
        _overall_min_accepted[c] += rate_min;
        _overall_avg_accepted[c] += rate_avg;
        _overall_max_accepted[c] += rate_max;
        _ComputeStats( _accepted_packets[c], &count_sum, &count_min, &count_max );
        rate_min = (double)count_min / time_delta;
        rate_sum = (double)count_sum / time_delta;
        rate_max = (double)count_max / time_delta;
        rate_avg = rate_sum / (double)_nodes;
        _overall_min_accepted_packets[c] += rate_min;
        _overall_avg_accepted_packets[c] += rate_avg;
        _overall_max_accepted_packets[c] += rate_max;

#ifdef TRACK_STALLS
        _ComputeStats(_buffer_busy_stalls[c], &count_sum);
        rate_sum = (double)count_sum / time_delta;
        rate_avg = rate_sum / (double)(_subnets*_routers);
        _overall_buffer_busy_stalls[c] += rate_avg;
        _ComputeStats(_buffer_conflict_stalls[c], &count_sum);
        rate_sum = (double)count_sum / time_delta;
        rate_avg = rate_sum / (double)(_subnets*_routers);
        _overall_buffer_conflict_stalls[c] += rate_avg;
        _ComputeStats(_buffer_full_stalls[c], &count_sum);
        rate_sum = (double)count_sum / time_delta;
        rate_avg = rate_sum / (double)(_subnets*_routers);
        _overall_buffer_full_stalls[c] += rate_avg;
        _ComputeStats(_buffer_reserved_stalls[c], &count_sum);
        rate_sum = (double)count_sum / time_delta;
        rate_avg = rate_sum / (double)(_subnets*_routers);
        _overall_buffer_reserved_stalls[c] += rate_avg;
        _ComputeStats(_crossbar_conflict_stalls[c], &count_sum);
        rate_sum = (double)count_sum / time_delta;
        rate_avg = rate_sum / (double)(_subnets*_routers);
        _overall_crossbar_conflict_stalls[c] += rate_avg;
#endif

    }
}

void TrafficManager::WriteStats(ostream & os) const {
  
    os << "%=================================" << endl;

    for(int c = 0; c < _classes; ++c) {
    
        if(_measure_stats[c] == 0) {
            continue;
        }
    
        //c+1 due to matlab array starting at 1
        os << "plat(" << c+1 << ") = " << _plat_stats[c]->Average() << ";" << endl
           << "plat_hist(" << c+1 << ",:) = " << *_plat_stats[c] << ";" << endl
           << "nlat(" << c+1 << ") = " << _nlat_stats[c]->Average() << ";" << endl
           << "nlat_hist(" << c+1 << ",:) = " << *_nlat_stats[c] << ";" << endl
           << "flat(" << c+1 << ") = " << _flat_stats[c]->Average() << ";" << endl
           << "flat_hist(" << c+1 << ",:) = " << *_flat_stats[c] << ";" << endl
           << "frag_hist(" << c+1 << ",:) = " << *_frag_stats[c] << ";" << endl
           << "hops(" << c+1 << ",:) = " << *_hop_stats[c] << ";" << endl;
        if(_pair_stats){
            os<< "pair_sent(" << c+1 << ",:) = [ ";
            for(int i = 0; i < _nodes; ++i) {
                for(int j = 0; j < _nodes; ++j) {
                    os << _pair_plat[c][i*_nodes+j]->NumSamples() << " ";
                }
            }
            os << "];" << endl
               << "pair_plat(" << c+1 << ",:) = [ ";
            for(int i = 0; i < _nodes; ++i) {
                for(int j = 0; j < _nodes; ++j) {
                    os << _pair_plat[c][i*_nodes+j]->Average( ) << " ";
                }
            }
            os << "];" << endl
               << "pair_nlat(" << c+1 << ",:) = [ ";
            for(int i = 0; i < _nodes; ++i) {
                for(int j = 0; j < _nodes; ++j) {
                    os << _pair_nlat[c][i*_nodes+j]->Average( ) << " ";
                }
            }
            os << "];" << endl
               << "pair_flat(" << c+1 << ",:) = [ ";
            for(int i = 0; i < _nodes; ++i) {
                for(int j = 0; j < _nodes; ++j) {
                    os << _pair_flat[c][i*_nodes+j]->Average( ) << " ";
                }
            }
        }

        double time_delta = (double)(_drain_time - _reset_time);

        os << "];" << endl
           << "sent_packets(" << c+1 << ",:) = [ ";
        for ( int d = 0; d < _nodes; ++d ) {
            os << (double)_sent_packets[c][d] / time_delta << " ";
        }
        os << "];" << endl
           << "accepted_packets(" << c+1 << ",:) = [ ";
        for ( int d = 0; d < _nodes; ++d ) {
            os << (double)_accepted_packets[c][d] / time_delta << " ";
        }
        os << "];" << endl
           << "sent_flits(" << c+1 << ",:) = [ ";
        for ( int d = 0; d < _nodes; ++d ) {
            os << (double)_sent_flits[c][d] / time_delta << " ";
        }
        os << "];" << endl
           << "accepted_flits(" << c+1 << ",:) = [ ";
        for ( int d = 0; d < _nodes; ++d ) {
            os << (double)_accepted_flits[c][d] / time_delta << " ";
        }
        os << "];" << endl
           << "sent_packet_size(" << c+1 << ",:) = [ ";
        for ( int d = 0; d < _nodes; ++d ) {
            os << (double)_sent_flits[c][d] / (double)_sent_packets[c][d] << " ";
        }
        os << "];" << endl
           << "accepted_packet_size(" << c+1 << ",:) = [ ";
        for ( int d = 0; d < _nodes; ++d ) {
            os << (double)_accepted_flits[c][d] / (double)_accepted_packets[c][d] << " ";
        }
        os << "];" << endl;
#ifdef TRACK_STALLS
        os << "buffer_busy_stalls(" << c+1 << ",:) = [ ";
        for ( int d = 0; d < _subnets*_routers; ++d ) {
            os << (double)_buffer_busy_stalls[c][d] / time_delta << " ";
        }
        os << "];" << endl
           << "buffer_conflict_stalls(" << c+1 << ",:) = [ ";
        for ( int d = 0; d < _subnets*_routers; ++d ) {
            os << (double)_buffer_conflict_stalls[c][d] / time_delta << " ";
        }
        os << "];" << endl
           << "buffer_full_stalls(" << c+1 << ",:) = [ ";
        for ( int d = 0; d < _subnets*_routers; ++d ) {
            os << (double)_buffer_full_stalls[c][d] / time_delta << " ";
        }
        os << "];" << endl
           << "buffer_reserved_stalls(" << c+1 << ",:) = [ ";
        for ( int d = 0; d < _subnets*_routers; ++d ) {
            os << (double)_buffer_reserved_stalls[c][d] / time_delta << " ";
        }
        os << "];" << endl
           << "crossbar_conflict_stalls(" << c+1 << ",:) = [ ";
        for ( int d = 0; d < _subnets*_routers; ++d ) {
            os << (double)_crossbar_conflict_stalls[c][d] / time_delta << " ";
        }
        os << "];" << endl;
#endif
    }
}

void TrafficManager::UpdateStats() {
#if defined(TRACK_FLOWS) || defined(TRACK_STALLS)
    for(int c = 0; c < _classes; ++c) {
#ifdef TRACK_FLOWS
        {
            char trail_char = (c == _classes - 1) ? '\n' : ',';
            if(_injected_flits_out) *_injected_flits_out << _injected_flits[c] << trail_char;
            _injected_flits[c].assign(_nodes, 0);
            if(_ejected_flits_out) *_ejected_flits_out << _ejected_flits[c] << trail_char;
            _ejected_flits[c].assign(_nodes, 0);
        }
#endif
        for(int subnet = 0; subnet < _subnets; ++subnet) {
#ifdef TRACK_FLOWS
            if(_outstanding_credits_out) *_outstanding_credits_out << _outstanding_credits[c][subnet] << ',';
            if(_stored_flits_out) *_stored_flits_out << vector<int>(_nodes, 0) << ',';
#endif
            for(int router = 0; router < _routers; ++router) {
                Router * const r = _router[subnet][router];
#ifdef TRACK_FLOWS
                char trail_char = 
                    ((router == _routers - 1) && (subnet == _subnets - 1) && (c == _classes - 1)) ? '\n' : ',';
                if(_received_flits_out) *_received_flits_out << r->GetReceivedFlits(c) << trail_char;
                if(_stored_flits_out) *_stored_flits_out << r->GetStoredFlits(c) << trail_char;
                if(_sent_flits_out) *_sent_flits_out << r->GetSentFlits(c) << trail_char;
                if(_outstanding_credits_out) *_outstanding_credits_out << r->GetOutstandingCredits(c) << trail_char;
                if(_active_packets_out) *_active_packets_out << r->GetActivePackets(c) << trail_char;
                r->ResetFlowStats(c);
#endif
#ifdef TRACK_STALLS
                _buffer_busy_stalls[c][subnet*_routers+router] += r->GetBufferBusyStalls(c);
                _buffer_conflict_stalls[c][subnet*_routers+router] += r->GetBufferConflictStalls(c);
                _buffer_full_stalls[c][subnet*_routers+router] += r->GetBufferFullStalls(c);
                _buffer_reserved_stalls[c][subnet*_routers+router] += r->GetBufferReservedStalls(c);
                _crossbar_conflict_stalls[c][subnet*_routers+router] += r->GetCrossbarConflictStalls(c);
                r->ResetStallStats(c);
#endif
            }
        }
    }
#ifdef TRACK_FLOWS
    if(_injected_flits_out) *_injected_flits_out << flush;
    if(_received_flits_out) *_received_flits_out << flush;
    if(_stored_flits_out) *_stored_flits_out << flush;
    if(_sent_flits_out) *_sent_flits_out << flush;
    if(_outstanding_credits_out) *_outstanding_credits_out << flush;
    if(_ejected_flits_out) *_ejected_flits_out << flush;
    if(_active_packets_out) *_active_packets_out << flush;
#endif
#endif

#ifdef TRACK_CREDITS
    for(int s = 0; s < _subnets; ++s) {
        for(int n = 0; n < _nodes; ++n) {
            BufferState const * const bs = _buf_states[n][0][s];
            for(int v = 0; v < _vcs; ++v) {
                if(_used_credits_out) *_used_credits_out << bs->OccupancyFor(v) << ',';
                if(_free_credits_out) *_free_credits_out << bs->AvailableFor(v) << ',';
                if(_max_credits_out) *_max_credits_out << bs->LimitFor(v) << ',';
            }
        }
        for(int r = 0; r < _routers; ++r) {
            Router const * const rtr = _router[s][r];
            char trail_char = 
                ((r == _routers - 1) && (s == _subnets - 1)) ? '\n' : ',';
            if(_used_credits_out) *_used_credits_out << rtr->UsedCredits() << trail_char;
            if(_free_credits_out) *_free_credits_out << rtr->FreeCredits() << trail_char;
            if(_max_credits_out) *_max_credits_out << rtr->MaxCredits() << trail_char;
        }
    }
    if(_used_credits_out) *_used_credits_out << flush;
    if(_free_credits_out) *_free_credits_out << flush;
    if(_max_credits_out) *_max_credits_out << flush;
#endif

}

void TrafficManager::DisplayStats(ostream & os) const {

    if ( _cp_enabled ) return;   // CP mode uses its own summary block

    for(int c = 0; c < _classes; ++c) {
    
        if(_measure_stats[c] == 0) {
            continue;
        }
    
        cout << "Class " << c << ":" << endl;
    
        cout 
            << "Packet latency average = " << _plat_stats[c]->Average() << endl
            << "\tminimum = " << _plat_stats[c]->Min() << endl
            << "\tmaximum = " << _plat_stats[c]->Max() << endl
            << "Network latency average = " << _nlat_stats[c]->Average() << endl
            << "\tminimum = " << _nlat_stats[c]->Min() << endl
            << "\tmaximum = " << _nlat_stats[c]->Max() << endl
            << "Slowest packet = " << _slowest_packet[c] << endl
            << "Flit latency average = " << _flat_stats[c]->Average() << endl
            << "\tminimum = " << _flat_stats[c]->Min() << endl
            << "\tmaximum = " << _flat_stats[c]->Max() << endl
            << "Slowest flit = " << _slowest_flit[c] << endl
            << "Fragmentation average = " << _frag_stats[c]->Average() << endl
            << "\tminimum = " << _frag_stats[c]->Min() << endl
            << "\tmaximum = " << _frag_stats[c]->Max() << endl;
    
        int count_sum, count_min, count_max;
        double rate_sum, rate_min, rate_max;
        double rate_avg;
        int sent_packets, sent_flits, accepted_packets, accepted_flits;
        int min_pos, max_pos;
        double time_delta = (double)(_time - _reset_time);
        _ComputeStats(_sent_packets[c], &count_sum, &count_min, &count_max, &min_pos, &max_pos);
        rate_sum = (double)count_sum / time_delta;
        rate_min = (double)count_min / time_delta;
        rate_max = (double)count_max / time_delta;
        rate_avg = rate_sum / (double)_nodes;
        sent_packets = count_sum;
        cout << "Injected packet rate average = " << rate_avg << endl
             << "\tminimum = " << rate_min 
             << " (at node " << min_pos << ")" << endl
             << "\tmaximum = " << rate_max
             << " (at node " << max_pos << ")" << endl;
        _ComputeStats(_accepted_packets[c], &count_sum, &count_min, &count_max, &min_pos, &max_pos);
        rate_sum = (double)count_sum / time_delta;
        rate_min = (double)count_min / time_delta;
        rate_max = (double)count_max / time_delta;
        rate_avg = rate_sum / (double)_nodes;
        accepted_packets = count_sum;
        cout << "Accepted packet rate average = " << rate_avg << endl
             << "\tminimum = " << rate_min 
             << " (at node " << min_pos << ")" << endl
             << "\tmaximum = " << rate_max
             << " (at node " << max_pos << ")" << endl;
        _ComputeStats(_sent_flits[c], &count_sum, &count_min, &count_max, &min_pos, &max_pos);
        rate_sum = (double)count_sum / time_delta;
        rate_min = (double)count_min / time_delta;
        rate_max = (double)count_max / time_delta;
        rate_avg = rate_sum / (double)_nodes;
        sent_flits = count_sum;
        cout << "Injected flit rate average = " << rate_avg << endl
             << "\tminimum = " << rate_min 
             << " (at node " << min_pos << ")" << endl
             << "\tmaximum = " << rate_max
             << " (at node " << max_pos << ")" << endl;
        _ComputeStats(_accepted_flits[c], &count_sum, &count_min, &count_max, &min_pos, &max_pos);
        rate_sum = (double)count_sum / time_delta;
        rate_min = (double)count_min / time_delta;
        rate_max = (double)count_max / time_delta;
        rate_avg = rate_sum / (double)_nodes;
        accepted_flits = count_sum;
        cout << "Accepted flit rate average= " << rate_avg << endl
             << "\tminimum = " << rate_min 
             << " (at node " << min_pos << ")" << endl
             << "\tmaximum = " << rate_max
             << " (at node " << max_pos << ")" << endl;
    
        cout << "Injected packet length average = " << (double)sent_flits / (double)sent_packets << endl
             << "Accepted packet length average = " << (double)accepted_flits / (double)accepted_packets << endl;

        cout << "Total in-flight flits = " << _total_in_flight_flits[c].size()
             << " (" << _measured_in_flight_flits[c].size() << " measured)"
             << endl;
    
#ifdef TRACK_STALLS
        _ComputeStats(_buffer_busy_stalls[c], &count_sum);
        rate_sum = (double)count_sum / time_delta;
        rate_avg = rate_sum / (double)(_subnets*_routers);
        os << "Buffer busy stall rate = " << rate_avg << endl;
        _ComputeStats(_buffer_conflict_stalls[c], &count_sum);
        rate_sum = (double)count_sum / time_delta;
        rate_avg = rate_sum / (double)(_subnets*_routers);
        os << "Buffer conflict stall rate = " << rate_avg << endl;
        _ComputeStats(_buffer_full_stalls[c], &count_sum);
        rate_sum = (double)count_sum / time_delta;
        rate_avg = rate_sum / (double)(_subnets*_routers);
        os << "Buffer full stall rate = " << rate_avg << endl;
        _ComputeStats(_buffer_reserved_stalls[c], &count_sum);
        rate_sum = (double)count_sum / time_delta;
        rate_avg = rate_sum / (double)(_subnets*_routers);
        os << "Buffer reserved stall rate = " << rate_avg << endl;
        _ComputeStats(_crossbar_conflict_stalls[c], &count_sum);
        rate_sum = (double)count_sum / time_delta;
        rate_avg = rate_sum / (double)(_subnets*_routers);
        os << "Crossbar conflict stall rate = " << rate_avg << endl;
#endif
    
    }
}

void TrafficManager::DisplayOverallStats( ostream & os ) const {

    if ( _cp_enabled ) return;   // CP mode uses its own summary block

    os << "====== Overall Traffic Statistics ======" << endl;
    for ( int c = 0; c < _classes; ++c ) {

        if(_measure_stats[c] == 0) {
            continue;
        }

        os << "====== Traffic class " << c << " ======" << endl;
    
        os << "Packet latency average = " << _overall_avg_plat[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tminimum = " << _overall_min_plat[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tmaximum = " << _overall_max_plat[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;

        os << "Network latency average = " << _overall_avg_nlat[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tminimum = " << _overall_min_nlat[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tmaximum = " << _overall_max_nlat[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;

        os << "Flit latency average = " << _overall_avg_flat[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tminimum = " << _overall_min_flat[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tmaximum = " << _overall_max_flat[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;

        os << "Fragmentation average = " << _overall_avg_frag[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tminimum = " << _overall_min_frag[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tmaximum = " << _overall_max_frag[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;

        os << "Injected packet rate average = " << _overall_avg_sent_packets[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tminimum = " << _overall_min_sent_packets[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tmaximum = " << _overall_max_sent_packets[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
    
        os << "Accepted packet rate average = " << _overall_avg_accepted_packets[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tminimum = " << _overall_min_accepted_packets[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tmaximum = " << _overall_max_accepted_packets[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;

        os << "Injected flit rate average = " << _overall_avg_sent[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tminimum = " << _overall_min_sent[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tmaximum = " << _overall_max_sent[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
    
        os << "Accepted flit rate average = " << _overall_avg_accepted[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tminimum = " << _overall_min_accepted[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
        os << "\tmaximum = " << _overall_max_accepted[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
    
        os << "Injected packet size average = " << _overall_avg_sent[c] / _overall_avg_sent_packets[c]
           << " (" << _total_sims << " samples)" << endl;

        os << "Accepted packet size average = " << _overall_avg_accepted[c] / _overall_avg_accepted_packets[c]
           << " (" << _total_sims << " samples)" << endl;
    
        os << "Hops average = " << _overall_hop_stats[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
    
#ifdef TRACK_STALLS
        os << "Buffer busy stall rate = " << (double)_overall_buffer_busy_stalls[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl
           << "Buffer conflict stall rate = " << (double)_overall_buffer_conflict_stalls[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl
           << "Buffer full stall rate = " << (double)_overall_buffer_full_stalls[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl
           << "Buffer reserved stall rate = " << (double)_overall_buffer_reserved_stalls[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl
           << "Crossbar conflict stall rate = " << (double)_overall_crossbar_conflict_stalls[c] / (double)_total_sims
           << " (" << _total_sims << " samples)" << endl;
#endif
    
    }
  
}

string TrafficManager::_OverallStatsCSV(int c) const
{
    ostringstream os;
    os << _traffic[c]
       << ',' << _use_read_write[c]
       << ',' << _load[c]
       << ',' << _overall_min_plat[c] / (double)_total_sims
       << ',' << _overall_avg_plat[c] / (double)_total_sims
       << ',' << _overall_max_plat[c] / (double)_total_sims
       << ',' << _overall_min_nlat[c] / (double)_total_sims
       << ',' << _overall_avg_nlat[c] / (double)_total_sims
       << ',' << _overall_max_nlat[c] / (double)_total_sims
       << ',' << _overall_min_flat[c] / (double)_total_sims
       << ',' << _overall_avg_flat[c] / (double)_total_sims
       << ',' << _overall_max_flat[c] / (double)_total_sims
       << ',' << _overall_min_frag[c] / (double)_total_sims
       << ',' << _overall_avg_frag[c] / (double)_total_sims
       << ',' << _overall_max_frag[c] / (double)_total_sims
       << ',' << _overall_min_sent_packets[c] / (double)_total_sims
       << ',' << _overall_avg_sent_packets[c] / (double)_total_sims
       << ',' << _overall_max_sent_packets[c] / (double)_total_sims
       << ',' << _overall_min_accepted_packets[c] / (double)_total_sims
       << ',' << _overall_avg_accepted_packets[c] / (double)_total_sims
       << ',' << _overall_max_accepted_packets[c] / (double)_total_sims
       << ',' << _overall_min_sent[c] / (double)_total_sims
       << ',' << _overall_avg_sent[c] / (double)_total_sims
       << ',' << _overall_max_sent[c] / (double)_total_sims
       << ',' << _overall_min_accepted[c] / (double)_total_sims
       << ',' << _overall_avg_accepted[c] / (double)_total_sims
       << ',' << _overall_max_accepted[c] / (double)_total_sims
       << ',' << _overall_avg_sent[c] / _overall_avg_sent_packets[c]
       << ',' << _overall_avg_accepted[c] / _overall_avg_accepted_packets[c]
       << ',' << _overall_hop_stats[c] / (double)_total_sims;

#ifdef TRACK_STALLS
    os << ',' << (double)_overall_buffer_busy_stalls[c] / (double)_total_sims
       << ',' << (double)_overall_buffer_conflict_stalls[c] / (double)_total_sims
       << ',' << (double)_overall_buffer_full_stalls[c] / (double)_total_sims
       << ',' << (double)_overall_buffer_reserved_stalls[c] / (double)_total_sims
       << ',' << (double)_overall_crossbar_conflict_stalls[c] / (double)_total_sims;
#endif

    return os.str();
}

void TrafficManager::DisplayOverallStatsCSV(ostream & os) const {
    for(int c = 0; c < _classes; ++c) {
        os << "results:" << c << ',' << _OverallStatsCSV() << endl;
    }
}

//read the watchlist
void TrafficManager::_LoadWatchList(const string & filename){
    ifstream watch_list;
    watch_list.open(filename.c_str());
  
    string line;
    if(watch_list.is_open()) {
        while(!watch_list.eof()) {
            getline(watch_list, line);
            if(line != "") {
                if(line[0] == 'p') {
                    _packets_to_watch.insert(atoi(line.c_str()+1));
                } else {
                    _flits_to_watch.insert(atoi(line.c_str()));
                }
            }
        }
    
    } else {
        Error("Unable to open flit watch file: " + filename);
    }
}

int TrafficManager::_GetNextPacketSize(int cl) const
{
    assert(cl >= 0 && cl < _classes);

    vector<int> const & psize = _packet_size[cl];
    int sizes = psize.size();

    if(sizes == 1) {
        return psize[0];
    }

    vector<int> const & prate = _packet_size_rate[cl];
    int max_val = _packet_size_max_val[cl];

    int pct = RandomInt(max_val);

    for(int i = 0; i < (sizes - 1); ++i) {
        int const limit = prate[i];
        if(limit > pct) {
            return psize[i];
        } else {
            pct -= limit;
        }
    }
    assert(prate.back() > pct);
    return psize.back();
}

double TrafficManager::_GetAveragePacketSize(int cl) const
{
    vector<int> const & psize = _packet_size[cl];
    int sizes = psize.size();
    if(sizes == 1) {
        return (double)psize[0];
    }
    vector<int> const & prate = _packet_size_rate[cl];
    int sum = 0;
    for(int i = 0; i < sizes; ++i) {
        sum += psize[i] * prate[i];
    }
    return (double)sum / (double)(_packet_size_max_val[cl] + 1);
}
