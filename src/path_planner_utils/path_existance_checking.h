#include <vector>
#include <iostream>
namespace pathexsitance{
    class state
    {
    public:
        state(double X, double Y){ value[0] = X; value[1]= Y;};
        state(){value[0]=0; value[1]=0;};
        ~state() = default;
        double Getx(){return value[0];};
        double Gety(){return value[1];};
        double* Getvalue(){return value;};
    protected:
        double value[2];
    };
    /** \brief Representation of a motion */
    class Motion
    {
    public:
        
        Motion() : mstate(nullptr), parent(nullptr), visited(0)
        {
        }
        Motion(state* istate,Motion* iparent):mstate(istate),parent(iparent),visited(0){};
        ~Motion() = default;

        /** \brief The state contained by the motion */
        state* mstate;

        /** \brief The parent motion in the exploration tree */
        Motion *parent;

        /** \brief The set of motions descending from the current motion */
        std::vector<Motion *> children;

        int visited;
    };

    class Path_checker
    {
    public:
        Path_checker(std::vector<std::vector<int>> &map,double imax_x, double imax_y, double resol):m_map(map),
            max_x(imax_x),max_y(imax_y), resolution(resol){};
        ~Path_checker(){
            // for(auto s: map_states){
            //     if(s) delete s;
            // }
            // for(auto motion: map_motion){
            //     if(motion) delete motion;
            // }
            // if(start_state) delete start_state;
            // if(end_state) delete end_state;
        };
        void create_tree_nodes();
        void set_start_goal(double* start, double* goal);
        bool isValid(state* State);
        void add_tree_edges();
        bool searchTree();
        void searchTree_recursive(std::vector<Motion*> &i_node, bool &path_exsistance);
    protected:
        std::vector<std::vector<int>> m_map;
        std::vector<state*> map_states;
        std::vector<Motion*> map_motion;
        double max_x,max_y,resolution;
        state* start_state;
        state* end_state;   
        int planning_plane;

    };
    


  
}