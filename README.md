# 2-Achsen-Delta

Der 2 Achsen Delta Roboter wird ein Programmierprojekt sein, welches einen Deltaroboter per Koordinatensystem von seinem jetzigen Punkt zu einem beliebigen Punkt X bewegt. Dieser Punkt wird mit einer X und einer Y Koordinate definiert.

## Muss Kriterien

Muss Kriterien sind solche, welche zu Projektende erfüllt werden müssen

### Ansteuerung über Numpad (Joel)
Die Positionseingabe wird über ein externes Numpad, welches am Arduino montiert ist, erfolgen.

### Positionierungs- und Ansteuerungs Algorithmus (Armin)
Der Deltaroboter soll von seiner jetzigen Position zu einer beliebigen Position X fahren. Dies sollte unabhängig von Genauigkeit, Geschwindigkeit, Linearität und Erreichbarkeit sein.

## Wunsch Kriterien

Wunsch Kriterien sind solche, welche zu Projektende nicht erfüllt werden müssen, jedoch wüsnchenswert wären.

### Linearität
Der Deltaroboter soll linear zwischen zwei Punkten verfahren können. Dies unabhängig von Geschwindigkeit.

### Motion Control
Der Deltaroboter sollte über einen Geschwindigkeitsparameter verfügen, bei welchem die Geschwindigkeit angegeben werden sollte. Zudem sollte der Roboter am Anfang und Ende jeweils Be- und Entschleunigen.

### Erreichbarkeit
Der Deltaroboter muss merken, wenn eine Position sich ausserhalb seines Arbeitsbereiches befindet und dies Rückmelden.
