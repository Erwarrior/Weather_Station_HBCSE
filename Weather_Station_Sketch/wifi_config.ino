
void InitWiFi()
{
  
  digitalWrite(esp_switch, LOW);
  Serial2.println("Initialising WiFi");
  delay(500);
  Serial1.flush();
  delay(20);
  Serial2.begin(115200);    // initialize serial for ESP module
  Serial2.println(" WiFi_begin");
  
  WiFi.init(&Serial1);    // initialize ESP module
  Serial2.println("WiFi_init()");
  
  if (WiFi.status() == WL_NO_SHIELD) 
  {
   
    Serial2.println("WiFi absent");
    // don't continue
    while (true);
  }
  else
  {
    printMacAddress();
  }
  
  delay(100);  
  Ap_connection();

  
}

void Ap_connection()
{
  listNetworks();
  delay(20);
  
  WIFI_AP = WiFi.SSID(signal_avail_global[0] );

  reconnect_counter = 0;
  while ( status != WL_CONNECTED) 
  {
    if(reconnect_counter >= 0 && reconnect_counter < 3)
      {
       WIFI_AP = WiFi.SSID(signal_avail_global[0] );
      }
      if(reconnect_counter >=3 && reconnect_counter <=5)
      {
        WIFI_AP = WiFi.SSID(signal_avail_global[1] );
      }
      if(reconnect_counter > 5)
      {
        resetFunc();
      }
    Serial2.print("Attemp to connect SSID: ");
    Serial2.println(WIFI_AP);
    status = WiFi.beginOn(WIFI_AP);
    delay(50);
    reconnect_counter++;
  }
  
  Serial2.print("Connected to AP: ");
  Serial2.println(WIFI_AP);
  
  Serial2.println("Obtaining IP: ");
  delay(100);
  
  IPAddress ip = WiFi.localIP();
  Serial2.print("IP Address: ");
  Serial2.println(ip);
}

void printMacAddress()
{
  // get your MAC address
  byte mac[6];
  WiFi.macAddress(mac);
  
  // print MAC address
  char buf[20];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
  Serial2.print("MAC address: ");
  Serial2.println(buf);
 
}

void listNetworks()
{
  
  // scan for nearby networks
  int numSsid = WiFi.scanNetworks();
  if (numSsid == -1) 
  {
    Serial2.println("No Wifi N/W");
    resetFunc();
  }

  // print the list of networks seen
  Serial2.print("No's of WiFi Signal:");
  Serial2.println(numSsid);

  // print the network number and name for each network found
  for (int thisNet = 0; thisNet < numSsid; thisNet++) 
  {
    Serial2.print(thisNet);
    Serial2.print(") ");
    Serial2.print(WiFi.SSID(thisNet));
    Serial2.print("\tSignal: ");
    Serial2.print(WiFi.RSSI(thisNet));
    Serial2.print(" dBm");
    Serial2.print("\tEncryption: ");
    printEncryptionType(WiFi.encryptionType(thisNet));    
  }

  unsigned int signal_counter = 0;
  Serial2.print("No of Open N/Ws: ");
  Serial2.println(count);
  unsigned int signal_avail[count];

  for (int thisNet = 0; thisNet < numSsid; thisNet++) 
  {
    if(WiFi.encryptionType(thisNet) == ENC_TYPE_NONE)
    {
      //Serial.println(thisNet);
      signal_avail[signal_counter] = thisNet;
      signal_counter++;
    }
  }
  Serial2.println("Open Networks");
  for (int sig = 0; sig < count; sig++) 
  {
    Serial2.print(sig);
    Serial2.print(") ");
    Serial2.print(WiFi.SSID(signal_avail[sig]));
    Serial2.print("\tSignal: ");
    Serial2.print(WiFi.RSSI(signal_avail[sig]));
    Serial2.println(" dBm");
  }

  int temp;
  for(int i=0;i<count;i++)
  {
    for(int j=i+1;j<count;++j)
    {
      if(WiFi.RSSI(signal_avail[i]) < WiFi.RSSI(signal_avail[j]))
      {
        temp = signal_avail[i];
        signal_avail[i] = signal_avail[j];
        signal_avail[j] = temp;
      }
    }
  }

  Serial.println("Sorted Networks");
  for (int sig = 0; sig < count; sig++) 
  {
    Serial2.print(sig);
    Serial2.print(") ");
    Serial2.print(WiFi.SSID(signal_avail[sig]));
    Serial2.print("\tSignal: ");
    Serial2.print(WiFi.RSSI(signal_avail[sig]));
    Serial2.println(" dBm");
  }

  Serial.println("New Array");
  for(int i = 0;i<3;i++)
  {
    signal_avail_global[i] = signal_avail[i];
    Serial.println(WiFi.SSID(signal_avail_global[i]));
  }
  
}
 
void printEncryptionType(int thisType) 
{
  // read the encryption type and print out the name
  
  switch (thisType) 
  {
    case ENC_TYPE_WEP:
      Serial2.print("WEP");
      break;
    case ENC_TYPE_WPA_PSK:
      Serial2.print("WPA_PSK");
      break;
    case ENC_TYPE_WPA2_PSK:
      Serial2.print("WPA2_PSK");
      break;
    case ENC_TYPE_WPA_WPA2_PSK:
      Serial2.print("WPA_WPA2_PSK");
      break;
    case ENC_TYPE_NONE:
      Serial2.print("None");
      count++; 
      break;
  }
  Serial2.println(" ");
}
