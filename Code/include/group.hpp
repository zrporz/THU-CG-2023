#ifndef GROUP_H
#define GROUP_H


#include "object3d.hpp"
#include "ray.hpp"
#include "hit.hpp"
#include <iostream>
#include <vector>

// TODO: Implement Group - add data structure to store a list of Object*
class Group : public Object3D {

public:

    Group() {

    }

    explicit Group (int num_objects):obj_list(num_objects) {

    }

    ~Group() override {

    }

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        bool is_intersect = 0;
        // std::cerr<<"here!"<<std::endl;
        for(auto it:obj_list){
            if(it&&it->intersect(r,h,tmin)){
                is_intersect = true;
            }
        }
        return is_intersect;
    }

    void addObject(int index, Object3D *obj) {
        obj_list.insert(obj_list.begin()+index,obj);
    }

    int getGroupSize() {
        return obj_list.size();
    }

private:
    std::vector<Object3D *> obj_list;
};

#endif
	
