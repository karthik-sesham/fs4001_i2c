#include <fs4001.h>

fs4001 fs = fs4001(true);

void setup() {
  Serial.begin(9600);  // start serial for output
  Serial.println("FS4001 test!");
  
  if (! fs.begin()) {
    Serial.println("Couldnt start");
    while(1);
  }
  Serial.println("FS4001 found!");
  Serial.println(fs.read_addr());
  Serial.println(fs.read_filter_depth());
  Serial.println(fs.read_sn().c_str());
  fs.write_response_time(65);
  Serial.println(fs.read_response_time());
  fs.write_gas_factor(250);
  Serial.println(fs.read_gas_factor());

}

void loop() {
  Serial.println(fs.read_flowrate());
  Serial.println();
  delay(1000);

}
