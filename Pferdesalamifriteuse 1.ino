


int opendist = 15;



//neger
//control motors

void OWfw(){

}
void NSfw(){

}
void OWbw(){

}
void NSbw(){

}
void OWx(){

}
void NSx(){

}

//relative driving

void driverelatively(int relativedirection,int direction){
  int directionnew = relativedirection + direction;
  if (directionnew > 3){
    directionnew -= 3;
  }
  
    
  if (directionnew == 0){
    NSfw();
    OWx();
  }
  else if (directionnew == 1){
    OWfw();
    NSx();
  }
  else if (directionnew == 2){
    NSbw();
    OWx();
  }
  else if (directionnew == 3){
    OWbw();
    NSx();
  }
  
  
}


// following functions for sensors

int getdistfront(){


}
int getdistright(){


}
int getdistback(){


}
int getdistleft(){


}


// following functions connect the sensors for ease of use



int getrelativedist(int relativesensor, int direction){ //0=front 1=right 2=back 3=left
  int directionnew = relativesensor + direction;
  if (directionnew > 3){
    directionnew -= 3;
  }
  
  
  if (directionnew == 0){
    return getdistfront();
  }
  if (directionnew == 1){
    return getdistright();
  }
  if (directionnew == 2){
    return getdistback();
  }
  if (directionnew == 3){
    return getdistleft();
  }

  
}


// figuring out what way to drive


int getnextstep(int currentdirection)
{
  
  if(getrelativedist(3,currentdirection) < opendist){
    driverelatively(3,currentdirection);
  }
  else if(getrelativedist(0,currentdirection) < opendist){
    driverelatively(0,currentdirection);
  }
  else if(getrelativedist(1,currentdirection) < opendist){
    driverelatively(1,currentdirection);
  }
  else{
    driverelatively(2,currentdirection);
  }
  
  
}


