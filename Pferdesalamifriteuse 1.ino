

// following functions for sensors

int getdistfront(){


}
int getdistright(){


}
int getdistback(){


}
int getdistleft(){


}

int getdist(int direction){
  if (direction == 0){
    return getdistfront();
  }
  if (direction == 1){
    return getdistright();
  }
  if (direction == 2){
    return getdistback();
  }
  if (direction == 3){
    return getdistleft();
  }

}


// following functions connect the sensors relatively to looking position

int getrelativedist(int relativesensor, int direction){ //0=front 1=right 2=back 3=left
  int directionnew = relativesensor + direction;
  if (directionnew > 3){
    directionnew -= 3
  }
  
  
  
}
