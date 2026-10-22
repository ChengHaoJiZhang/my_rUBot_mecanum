Colcon build - per compilar el projecte en el software que es posa
4 arxius configuren tot el projecte de ros 2

Quins nodes amb el bringup?

Només podem veure els nodes que tenim (domainaty)

websocket: servei que fa controlar robot des de internet

nodes publiquen a traves de topics, topics associats a nodes alla es publiquen informacio. qualsevol connectat a nodes de publicacio pot accedir. el grup x nomes te acces node x

topic list: quisn topics en la rasberry pero no a que topic esta cada node. Ens hauriem de subscriure per sabrho o comandes de terminal.

rviz2: app que ens ensenya el problema d'amunt

telopnoseque: suscribir i controlar robot package i despres codi

a l'ordinado un script pythong suscriber es comnica amb nodes del driver i llegeix diversos topcis d auqest node odometry velocity etc envia comandaments al node.

generarem .py que son suscribers. faran servir dels nodes del robot que publica genera la raseberry y que veiem des del pc
- per exemple pararem lidar. quan ho tinguem farem colcon build de compilar. 

websocket rasberry servei internet amb sockets estar en la mateixa wifi movil ordinador
la pagina web que esta publicant el robot+

hem de fer el bringup

-usb
-12volts
-clon repo
-comprovem els ndes (5) actius i topcis (10-15)
    si alguna vegada el lidar, un reboot de tot 

- s'evalua els commits que tots els integrants feu.
    els integrants han de fer commit al director.

    que cadascu faci un commit contra el repositori del director

Si per algun casual i no fa commit i push i algu far commit i push 

fatal error you are behind - preguntar al chat 

git permet fer arxius dos a la vegada femnt branques. MAI MAI dos personas a la mateixa vegada al mateix arxiu si dos persones han fet una mateixa mod al mateix arxiu obrira una branca -> problema no sabem treballar amb branques 

CANVIS PER FER 