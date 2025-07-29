#include <CircularBuffer.hpp>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

CircularBuffer<int,33> inp;
CircularBuffer<int,33> lpfout;
CircularBuffer<int,33> hpfout;
CircularBuffer<int,33> deriv;
CircularBuffer<int,33> squar;
CircularBuffer<int,33> integ;
CircularBuffer<int,33> intervals;

LiquidCrystal_I2C lcd(0x27,20,4);

int raw, filt, pass, result, ct=0; 
unsigned long t, timeNow, timePassed;
int f=0;
double sec, Hz, BPM;

double rmssd = 0.0, sdnn = 0.0;


void setup() {
  lcd.init();                      // initialize the lcd 
  lcd.backlight();

  pinMode(13, INPUT);
  Serial.begin(115200);
  for (int i = 0; i <= 32; i++) {
    inp.unshift(0);
    lpfout.unshift(0);
    hpfout.unshift(0);
    deriv.unshift(0);
    squar.unshift(0);
    integ.unshift(0);
  }

}

void loop() {
  t=micros();

  while(t%5000==0){

  raw = analogRead(13);
  inp.unshift(raw);

  //Lowpass filter
  filt = 2*lpfout[0] - lpfout[1] + inp[0] - 2*inp[6] + inp[12];
  lpfout.unshift(filt);  // this becomes lpfout[0] for the next loop
  
  //Highpass filter
  filt = -hpfout[0] - lpfout[0] + 32*lpfout[16] + lpfout[32];
  hpfout.unshift(filt);

  //Derivative filter
  filt = hpfout[-2] + 2*hpfout[-1] - 2*hpfout[1] - hpfout[2];
  filt = 200*filt;
  filt = filt >> 3;
  deriv.unshift(filt);

  //Squaring filter
  filt = (deriv[0])^2;
  squar.unshift(filt);

  //Moving Window Integration
  filt = squar[31]+squar[30]+squar[29]+squar[28]+squar[27]+squar[26]+squar[25]+squar[24]+
         squar[23]+squar[22]+squar[21]+squar[20]+squar[19]+squar[18]+squar[17]+squar[16]+
         squar[15]+squar[14]+squar[13]+squar[12]+squar[11]+squar[10]+squar[9] +squar[8] +
         squar[7] +squar[6] +squar[5] +squar[4] +squar[3] +squar[2] +squar[1] +squar[0];
  filt = filt >> 4;
  integ.unshift(filt);
  result = filt;


  // Threshold Logic: This is not an adaptive thresholding logic but a simple and static one   
  if(result>2000000 && f==0) {
  timeNow = millis();
  f=1;}
  if(result<2000000 && f==1){
  f=2;}
  if(result>2000000 && f==2){
  timePassed = millis() - timeNow;
  f=0;}
  sec = (double(timePassed)/1000);
  Hz = (double(1)/sec);
  BPM = (60*Hz);
  intervals.unshift(sec);


  //The statistics derived from the BPM have been removed in this version of the code due to some errors and deviations from expected values
    
  Serial.print(BPM);
  Serial.print(" ");
  Serial.print(raw);
  Serial.print(" ");
  Serial.print(result);
  Serial.println();

  }
}
