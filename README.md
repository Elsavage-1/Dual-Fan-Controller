# Dual-Fan-Controller
Created for the purpose of syncing the RPM speeds of the Sycthe Flex II and Flex II slim models on an arduino. 
This gets rid of a weird noise frequency when both fans are stacked near each other. 

An Arduino Uno R3 was used for this project

PWM signal is directly connected to the pins. 

A pontimeter is hooked up to A0 input for RPM controller

Additional components were used on RPM signal to help get rid of noise:

Component lay used to filter noise, each RPM signal uses the following inpuit:

```mermaid
graph TB
    %% Invisible link to position RPM to the left of the subgraph
   RPM ~~~~subGraph0
   RPM["PC FAN RPM<br>Green Wire<br>(Tachometer output)"]

    subgraph subGraph0["Fan Tachometer Input Circuit"]
        FAN_TACH["Tach Signal Line"]
        R_PU["R1: 10kΩ<br>Pull-up Resistor"]
        R_FILT["R2: 100Ω<br>Filter Resistor"]
        C_FILT["C1: 0.1µF<br>Ceramic Capacitor<br>Noise Filter"]
        MCU_PIN["Arduino input <br>(ATmega328P Pin 2)"]
    end

    V5["+5V"] --> R_PU
    RPM --> FAN_TACH
    R_PU --> FAN_TACH
    FAN_TACH --> R_FILT
    R_FILT --> C_FILT & MCU_PIN
    C_FILT -.-> GND["GND"]

    RPM@{ shape: rounded}
    V5:::power
    GND:::power
    RPM:::rpmSignal
    FAN_TACH:::signal
    R_PU:::component
    R_FILT:::component
    C_FILT:::component
    MCU_PIN:::micro

    classDef power fill:#ffcccc
    classDef signal fill:#ccffcc
    classDef component fill:#ccccff
    classDef micro fill:#ffffcc
    classDef rpmSignal fill:#00D100
```

a 12v power supply with a 5.5x2.1 barrel jack is used,  is used and split out to power each item:
PC fans use a DC power to PC fan adapter, a Y cable splits that out to both fans. Fans are connected with jumper cables

4-Pin PWM PC fans use the following pin outs, and generally use these colors:
Pin 1: GND ( black )
Pin 2: PWR ( yellow )
Pin 3: RPM ( Green )
Pin 4: PWM ( Blue )

In the case of all black wires, Pin 1, GND, is often stripped in gray. The PC fan connector has the notch directly on the edge connector as well for the gnd pin as, 4pin connectors are 3pin compatible the other notch is offset and not flush with the edge of the connector. 

```mermaid
graph TB
    %% Invisible link to position 12V to the left of the subgraph
    V12 ~~~~subGraph0
    V12["12V DC INPUT<br>(Power Supply)"]

    subgraph subGraph0["Power Distribution Circuit"]
        FAN1_PWR["Fan 1 Power<br>Red Wire"]
        FAN2_PWR["Fan 2 Power<br>Red Wire"]
        ARD_PWR["Arduino<br>Barrel Jack"]
    end


    V12 --> FAN1_PWR & FAN2_PWR
    V12 --> ARD_PWR
    FAN1_PWR --> GND["GND"]
    FAN2_PWR --> GND
    ARD_PWR --> GND


    V12@{ shape: rounded}
    V12:::power
    FAN1_PWR:::yellow
    FAN2_PWR:::yellow
    ARD_PWR:::yellow

    GND:::power

    classDef power fill:#ffcccc
    classDef yellow fill:#ffffcc
```
