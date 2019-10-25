#include "path_existance_checking.h"

namespace pe = pathexsitance;

// pe::Path_checker(std::vector<std::vector<std::vector<char>>> map)

void pe::Path_checker::create_tree_nodes(){
	int num_of_states_x = resolution * max_x;
	int num_of_states_y = resolution * max_y;
	// map_states.resize(num_of_states_x*num_of_states_y,nullptr);
	for(int i=0; i< num_of_states_x; ++i){
		for(int j=0; j< num_of_states_y; ++j){
			state* c_state = new state(i*resolution,j*resolution);
			map_states.push_back(c_state);
			// std::cout<<"Create node: i "<<i<<" j: "<<j<<"State x "<<c_state->Getx()<<" Y "<<c_state->Gety()<<" map state "
			// std::cout<<m_map[i][j];
		}
		// std::cout<<std::endl;
	}
}

void pe::Path_checker::set_start_goal(double* start, double* goal){

	start_state = new state(start[0],start[1]);
	end_state = new state(goal[0],goal[1]);
	if(!isValid(start_state)) std::cout<<" ERROR: start state invalid, may be too close to obstacle"<<std::endl;
	if(!isValid(end_state)) std::cout<<" ERROR: end state invalid, may be too close to obstacle"<<std::endl;

}

void pe::Path_checker::add_tree_edges(){
	int num_of_states_x = resolution * max_x;
	int num_of_states_y = resolution * max_y;
	map_motion.resize(num_of_states_x*num_of_states_y,nullptr);
	for(int i=1; i< num_of_states_x-1; ++i){
		for(int j=1; j< num_of_states_y-1; ++j){
			// std::cout<<" state x "<<i<<" y "<<j<<" map index "<<(i*num_of_states_y)+j<<" validity "
			// <<isValid(map_states[(i*num_of_states_y)+j])<<std::endl;
			Motion* p_motion;
			if(!map_motion[(i*num_of_states_x)+j]){
				p_motion = new Motion(map_states[(i*num_of_states_x)+j],nullptr);
				map_motion[(i*num_of_states_x)+j] = p_motion;
			}
			else
				p_motion = map_motion[(i*num_of_states_x)+j];
			if(isValid(map_states[(i*num_of_states_x)+j])){
				for(int ii=-1; ii < 2; ++ii){
			    	for(int jj=-1; jj < 2;++jj){
			    		if( !(ii == 0 && jj ==0 ) ){
			    			if(isValid(map_states[((i+ii)*num_of_states_x)+j+jj])){
			    				Motion* m_motion=nullptr;
			    				if(!map_motion[((i+ii)*num_of_states_x)+j+jj]){
			    					m_motion = new Motion(map_states[((i+ii)*num_of_states_x)+j+jj],p_motion);
			    					map_motion[( (i+ii)*num_of_states_x)+j+jj]= m_motion;
			    				}
			    				else{
			    					m_motion = map_motion[((i+ii)*num_of_states_x)+j+jj];
			    				}
		    					p_motion->children.push_back(m_motion);
		    					// std::cout<<"Adding edge: i "<< i<<" j: "<<j<<" child x "<<i+ii<<" Y "<<j+jj<<std::endl;
			    			}
			    		}
			    	}
			    }

			    // for(auto mot:p_motion->children){
			    // 	std::cout<<" C X: "<<(*mot).mstate->Getx()<<" C Y: "<<(*mot).mstate->Gety()<<std::endl;
			    // }
			}
		}
	}
}

bool pe::Path_checker::isValid(state* State)
{

    for(int i=-1; i < 2; ++i){
    	for(int j=-1; j < 2;++j){

    		if(State->Getvalue()[0]+i < m_map.size() &&
    			State->Getvalue()[0]+i >= 0 &&
    			State->Getvalue()[1]+j < m_map[State->Getvalue()[0]+i].size() &&
    			State->Getvalue()[1]+j >= 0){
	    		if(m_map[State->Getvalue()[0]+i][State->Getvalue()[1]+j]) return false;
    		}
    		else{
    			return false;
    		}
    	}
    }
	return true;
}

void pe::Path_checker::searchTree_recursive(std::vector<Motion*> &i_node, bool &path_exsistance){
	if(i_node.size()>0){
		for(std::vector<Motion*>::iterator i=i_node.begin(); i != i_node.end(); ++i){
			if(!(*i)->visited){
				if( (*i)->mstate->Getx() == end_state->Getx() && (*i)->mstate->Gety() == end_state->Gety()){
					path_exsistance = true;
					// std::cout<<"ONe PATH FOUND !!!!!!!!!!!!!!!"<<std::endl;
				}
				(*i)->visited=1;
				// std::cout<<"("<<(*i)->mstate->Getx()<<","<<(*i)->mstate->Gety()<<")-> size "<<i_node.size()<<std::endl;
				// std::cout<<"("<<(*i)->mstate->Getx()<<","<<(*i)->mstate->Gety()<<")->";
				if((*i)->children.size()>0) searchTree_recursive( (*i)->children, path_exsistance);
			}
		}
		// std::cout<<std::endl;
	}
}

bool pe::Path_checker::searchTree(){
	bool pathexsitance = false;
	int num_of_states_x = resolution * max_x;
	int num_of_states_y = resolution * max_y;
	if(map_motion[(start_state->Getx()*num_of_states_x)+start_state->Gety()]){
		searchTree_recursive(map_motion[(start_state->Getx()*num_of_states_x)+start_state->Gety()]->children,pathexsitance);	
	}
	else std::cout<<" ERROR: start state invalid, may be too close to obstacle"<<std::endl;
	
	// if(pathexsitance) std::cout<<"PATH exsists"<<std::endl;
	// else std::cout<<"PATH does not exsist "<<std::endl;

	return pathexsitance;
}
