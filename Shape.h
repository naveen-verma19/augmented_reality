//
// Created by Naveen Verma on 3/15/21.
//

#ifndef PROJ4_SHAPE_H
#define PROJ4_SHAPE_H

/**
 * This interface has al defined 3d shapes for our AR PLAY mode
 * It helps very much to keep a vector of shapes and just modify the x,y,z values on user keypress without worrying about the shape type
 */
class Shape{
public:
    //the current x,y,z for shape i.e center for cone,cylinder,sphere and corner for cube
    float x;
    float y;
    float z;
    float r; //radius or side for cube
    float h; //height of cone and cylinder

    PLANE p; //the current plane of base for cylinder,cone, cube, sphere doesnt have a plane
    int color_idx; //which color do they have
    bool marked; //is this shape currently selected for modification
    std::vector<cv::Scalar> colors_set
            {
                    cv::Scalar(0, 0, 255),
                    cv::Scalar(255, 0, 0),
                    cv::Scalar(0, 255, 0),
                    cv::Scalar(150, 0, 0),
                    cv::Scalar(0, 150, 0),
                    cv::Scalar(0, 0, 150),
                    cv::Scalar(255, 255, 0),
                    cv::Scalar(0, 255, 255),
                    cv::Scalar(255, 0, 255),
            };
    Shape(float x, float y,float z, float r){
        this->x=x;
        this->y=y;
        this->z=z;
        this->r=r;
        this->color_idx=0;
        this->marked= false;
    }
    //abstract
    virtual void draw(Drawer* drawer){
    };
};


//for below shapes
//draw uses the Drawer object to draw. the main method keeps reference of drawer object which remains same for a frame

class Sphere: public Shape{
public:
    Sphere(float x1, float y1, float z1, float r1) : Shape(x1, y1, z1, r1) {
    }

    void draw(Drawer* drawer) override{
        int actual_color=this->marked?1:this->color_idx; //if marked make it blue
        drawer->drawSphere(this->x,this->y,this->z,this->r, colors_set[actual_color]);
    }
};

class Cube: public Shape{
public:
    Cube(float x1, float y1, float z1, float r1) : Shape(x1, y1, z1, r1) {
        this->p=XY;
    }
    void draw(Drawer* drawer) override{
        int actual_color=this->marked?1:this->color_idx;
        drawer->drawCube(this->x,this->y,this->z,this->r, colors_set[actual_color],this->p);
    }
};

class Cylinder: public Shape{
public:
    Cylinder(float x, float y, float z, float r, float h) : Shape(x, y, z, r) {
        this->h=h;
        this->p=XY;
    }
    void draw(Drawer* drawer) override{
        int actual_color=this->marked?1:this->color_idx;
        drawer->drawCylinder(this->x,this->y,this->z,this->r,this->h,this->p,colors_set[actual_color]);
    }
};

class Cone: public Shape{
public:
    Cone(float x1, float y1, float z1, float r1, float h) : Shape(x1, y1, z1, r1) {
        this->h=h;
        this->p=XY;

    }
    void draw(Drawer* drawer) override{
        int actual_color=this->marked?1:this->color_idx;
        drawer->drawCone(this->x,this->y,this->z,this->r,this->h,this->p,colors_set[actual_color]);
    }
};


#endif //PROJ4_SHAPE_H
