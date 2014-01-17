// Buffer to store incoming commands from serial port
 char data [21];
 int number_of_bytes_received;                                                                                                   
int relay = 13;
int green = 4;
int blue = 5;
//int red = 3;
void setup() {
    Serial.begin(57600);
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
        number_of_bytes_received = Serial.readBytesUntil (84,data,20); // read bytes (max. 20) from buffer, untill <CR> (13). store bytes in data. count the bytes recieved.
       data[number_of_bytes_received] = 0; // add a 0 terminator to the char array
      
    }
    bool result = strcmp (data, "OFFRELAY");
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
if(result == false){
digitalWrite(relay,HIGH);
digitalWrite(green,LOW); 
digitalWrite(blue,HIGH); 
}
else
{
digitalWrite(green,HIGH);
digitalWrite(blue,LOW); 
}
}
