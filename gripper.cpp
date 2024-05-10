// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>
#include <kdl/treefksolverpos_recursive.hpp>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include "kdl_parser/kdl_parser.hpp"
//#include <String.h>
 
using namespace KDL;
using namespace std;
 
int main( int argc, char** argv )
{
    //Definition of a kinematic chain & add segments to the chain
   
   // chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,1.020))));
    //chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.480))));
    //chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.645))));
    //chain.addSegment(Segment(Joint(Joint::RotZ)));
   // chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.120))));
    //chain.addSegment(Segment(Joint(Joint::RotZ)));
    
    //take input from terminal
     if (argc < 2) {
    std::cerr << "Expect xml file to parse" << std::endl;
    return -1;
  }
  urdf::ModelInterfaceSharedPtr robot_model = urdf::parseURDFFile(argv[1]);
  if (!robot_model) {
    std::cerr << "Could not generate robot model" << std::endl;
    return false;
  }
  KDL::Chain chain;
 float a[3];
 char data;
 int r;
  KDL::Tree my_tree;
  if (!kdl_parser::treeFromUrdfModel(*robot_model, my_tree)) {
    std::cerr << "Could not extract kdl tree" << std::endl;
    return false;
  }
  my_tree.getChain("base_link","tcp",chain);
    // Create solver based on kinematic chain
   ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
   // KDL::TreeFkSolverPos_recursive fksolver = KDL::TreeFkSolverPos_recursive(my_tree);
 
 
    std::cout<<"Is there end effector?"<<endl;
    scanf("%c",&data);
    
    // Create joint array
    if(data =='y'){
    	
    	cout<<"Enter the position [x,y,z]:"<<endl;
    	for(int i=0;i<3;i++){
    		scanf("%e",&a[i]);
    	}
    	cout<<"enter the axis of rot: 1->X,2->Y,3->Z,4->NONE"<<endl;
    	scanf("%d",&r);
    }
    if(r==1){
    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(a[0],a[1],a[2]))));
    }
    else if (r==2){
    chain.addSegment(Segment(Joint(Joint::RotY),Frame(Vector(a[0],a[1],a[2]))));
    }
    else if (r==3){
    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(a[0],a[1],a[2]))));
    }
    else if (r==4){
    chain.addSegment(Segment(Joint(Joint::None),Frame(Vector(a[0],a[1],a[2]))));
    }
    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray jointpositions = JntArray(nj);
 
    
    // Assign some values to the joint positions
    for(unsigned int i=0;i<nj;i++){
        float myinput;
        printf ("Enter the position of joint %i: ",i);
        scanf ("%e",&myinput);
        jointpositions(i)=(double)myinput;
    }
 
    // Create the frame that will contain the results
    KDL::Frame cartpos;    
    
    // Calculate forward position kinematics
    bool kinematics_status;
    kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
    if(kinematics_status>=0){
        std::cout << cartpos <<std::endl;
        printf("%s \n","Succes, thanks KDL!");
    }else{
        printf("%s \n","Error: could not calculate forward kinematics :(");
    }
}
