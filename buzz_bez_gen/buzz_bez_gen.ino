void setup() {
  // put your setup code here, to run once:
  pinMode(A5, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
      int czestotliwosc;
      for (czestotliwosc = 31; czestotliwosc < 9535; czestotliwosc++) { 
        tone(A5, czestotliwosc); //Wygeneruj sygnał o częstotliwości 1000Hz na pinie A5  
      }
}
