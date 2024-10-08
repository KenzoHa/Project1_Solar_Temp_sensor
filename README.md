# Temperature & solar sensor for crypto mining-rig

## Beschrijving
Dit project optimaliseert een crypto-mining-rig door deze alleen te laten werken bij voldoende zonlicht, en in een ruimte die niet te warm of koud is. 

## Features
- **LDR / Lichtsensor**: monitoren of er voldoende zonlicht schijnt.
- **Temperatuursensor (DS18B20)**: Monitoren van omgevingstemperatuur.
- **RBG-LED-indicator**:
  - Geel: Voldoende zonlicht 
  - Paars: Onvoldoende zonlicht 
  - Rood: Te hoge temperatuur (> 25°C)
  - Blauw: Te lage temperatuur (< 14°C)
- **Drukknop**: Handmatige meting (zowel temperatuur als licht).
- **Bluetooth-verbinding**: Connectiviteit met mobiele apparaten (behalve iOS-apparaten).
- **Webpagina**: Via een webpagina kan data (mining-rig status) geraadpleegd worden. 

## Installatie
1. Benodigdheden: 
  •	Printplaat 
  •	1x Firebeetle ESP32 – E
  •	1x Lichtsensor (LDR) + 1x weerstand 100K Ω (pull-down)
  •	1x RBG-LED + 3x weerstand 220 Ω		
  •	1x drukknop + 1x weerstand 10K Ω (pull-down)
  •	1x groene LED + 1x weerstand 220 Ω	
  •	1x DS18B20 Digital Temperature Sensor + 1x Gravity: terminal sensor 
2. Aansluiten. 
3. Upload de code naar je FireBeetle ESP32-E.

## Toekomstige Verbeteringen
- Integratie van ActivationSensor met de mining-rig via WiFi. (of LAN?)
- Webpagina verbeteren.

## Contact
Voor vragen: [kenzo.haemerlinck@student.kdg.be].
