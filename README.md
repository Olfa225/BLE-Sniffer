Ce projet de noeud BLE (Bluetooth Low Energy) vise à améliorer la qualité de vie des personnes en surveillant des informations vitales et en assurant un contrôle domotique simple. Le système utilise une carte ESP32-C3 comme périphérique BLE, communiquant avec une application mobile, pour collecter et afficher des données de santé et interagir avec des équipements environnementaux.

Fonctionnalités principales
1. Surveillance des signes vitaux
Capteur de fréquence cardiaque : Un capteur de fréquence cardiaque est intégré pour mesurer en temps réel les battements du cœur de l'utilisateur.
Affichage des données : Les données de fréquence cardiaque sont affichées sur un écran LCD localement et également transmises à l'application mobile nRF Connect, qui agit comme le dispositif central. Cela permet aux proches et aux soignants de surveiller à distance des informations essentielles sur la santé.
2. Contrôle de l’environnement par servo-moteur
Commande de servo-moteur via BLE : Grâce à la connectivité BLE, l'application mobile nRF Connect peut contrôler un servo-moteur relié au noeud ESP32-C3.
Application pratique : Le servo-moteur peut, par exemple, ajuster les stores des fenêtres pour réguler la luminosité dans la pièce, en fonction des besoins du patient (plus ou moins de lumière selon son confort).
3. Communication Bluetooth Low Energy (BLE)
ESP32-C3 comme périphérique : La carte ESP32-C3 est configurée en tant que périphérique BLE, chargé de transmettre les données de santé et de recevoir les commandes de l'application centrale.
nRF Connect comme central : L'application nRF Connect agit en tant que dispositif central BLE, permettant une gestion bidirectionnelle des données et des commandes. Cela assure une communication en temps réel, fiable et à faible consommation énergétique entre les différents composants du système.
Aspects avancés du projet
1. Accessibilité et bien-être des patients
Suivi à distance : En utilisant l'application nRF Connect comme central, le système offre une surveillance à distance des données vitales et un contrôle environnemental simplifié pour améliorer le confort des patients.
Personnalisation : Le système peut être adapté à d'autres capteurs de santé ou dispositifs de contrôle, selon les besoins spécifiques de l'utilisateur.
2. Avantages de la technologie BLE
Faible consommation d'énergie : La technologie BLE est idéale pour les appareils portables et les capteurs médicaux, car elle permet une transmission de données efficace sans épuiser rapidement la batterie.
Interopérabilité : Le choix de l'application nRF Connect comme central BLE offre une compatibilité avec de nombreux dispositifs, simplifiant la configuration et la gestion du système.

