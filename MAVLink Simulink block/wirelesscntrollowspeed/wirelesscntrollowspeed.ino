// Buffer to store incoming commands from serial port
 char data [9];
 int number_of_bytes_received;                                                                                                   
int relay = 13;
int green = 4;
int blue = 5;
//int red = 3;
String stringOne;
void setup() {
    Serial.begin(9600);
    //Serial.println("Serial conection started, waiting for instructions...");
    
     // set the digital pin as output:
  pinMode(relay, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT);  
}
void loop() 
{     
    if(Serial.available() > 0)
    {
       number_of_bytes_received = Serial.readBytesUntil (72,data,8); // read bytes (max. 20) from buffer, untill <CR> (13). store bytes in data. count the bytes recieved.
       data[number_of_bytes_received] = 0; // add a 0 terminator to the char array
      
    }
    stringOne = "OFFRELAY";
    
    
/*    if (result == true)
 {
  // Serial.println("data does not matches whatever");
   digitalWrite(relay, LOW);
 } 
/else 
 {
  // Serial.println("data match whatever");
   digitalWrite(relay, HIGH);
 }*/
if(stringOne.equals(data)){
digitalWrite(relay,HIGH);
digitalWrite(green,LOW); 
digitalWrite(blue,HIGH); 
}
else
{
  digitalWrite(relay,LOW);
digitalWrite(green,HIGH);
digitalWrite(blue,LOW); 
}
//delay(1000);
}
