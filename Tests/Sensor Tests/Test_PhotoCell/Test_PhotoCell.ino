int readvalue;
const int analogInPinOne = A0;
const int analogInPinTwo = A1;
const int analogInPinThree = A2;

const int cl_0 = 65;  //black
const int cl_1 = 75;  //dark green
const int cl_2 = 130; //normal green
const int cl_3 = 150; //light green
const int cl_4 = 200; //white

int trs_1 = (cl_0 + cl_1)/2;
int trs_2 = (cl_1 + cl_2)/2;
int trs_3 = (cl_2 + cl_3)/2;
int trs_4 = (cl_3 + cl_4)/2;

void setup()
{
  Serial.begin(9600);
}
void loop()
{
  readvalue = analogRead(analogInPinOne);
  Serial.print(readvalue);
  Serial.print("\t");
  readvalue = analogRead(analogInPinTwo);
  Serial.print(readvalue);
  Serial.print("\t");
  readvalue = analogRead(analogInPinThree);
  Serial.print(readvalue);
  Serial.print("\n");
  
  /*if (readvalue < trs_1)
    Serial.print(" Black");
  else if (readvalue >= trs_1 && readvalue < trs_2)
    Serial.print(" Dark Green");
  else if (readvalue >= trs_2 && readvalue < trs_3)
    Serial.print(" Normal Green");
  else if (readvalue >= trs_3 && readvalue < trs_4)
    Serial.print(" Light Green");
  else if (readvalue >= trs_4)
    Serial.print(" White");
  Serial.print("\n");*/
}
