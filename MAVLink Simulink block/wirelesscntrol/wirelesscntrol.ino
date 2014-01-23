// Buffer to store incoming commands from serial port
 char data [21];
 int number_of_bytes_received;                                                                                                   
int relay = 13;
int green = 4;
int blue = 5;
//int red = 3;
int ch6;
int ch5;
void setup() {
    Serial.begin(9600);
    //Serial.println("Serial conection started, waiting for instructions...");
    
     // set the digital pin as output:
  pinMode(relay, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue, OUTPUT); 
   pinMode(10, INPUT); 
   pinMode(11, INPUT);
}
void loop() 
{     
    ch6 = pulseIn(10, HIGH, 25000);
    ch5 = pulseIn(11, HIGH, 25000);
if(ch6<1500)
{digitalWrite(relay, HIGH);}

if(ch6>1500)
{
delay(3000);
  digitalWrite(relay, LOW);
}

  delay(100);
}
