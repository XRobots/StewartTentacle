int thresholdStick (int pos) {
  
    // get zero centre position
    pos = pos - 512;  

    // threshold value for control sticks 
    if (pos > 50) {
      pos = pos -50;
    }
    else if (pos < -50) {
      pos = pos +50;
    }
    else {
      pos = 0;
    }

    pos = map(pos, -512,512,-255,255);
    pos = constrain(pos,-255,255);

    return pos;
}



// motion filter to filter motions and compliance

float filter(float prevValue, float currentValue, int filter) {  
  float lengthFiltered =  (prevValue + (currentValue * filter)) / (filter + 1);  
  return lengthFiltered;  
}

