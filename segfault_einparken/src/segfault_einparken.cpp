#include "std_msgs/String.h"
#include <math.h>
#include "sstream"
#include <iostream>
#include <utility>
#include <ros/ros.h>
#include <pses_basis/SensorData.h>
#include <pses_basis/Command.h>
#include <pses_basis/CarInfo.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <functional>

// Berechnet den Durchschnitt aus drei gemessenen Sensorwerten
// iterator -> Zaehler der bis drei zaehlt und dann auf wieder auf null springt
// arr -> das Array das mit iterator angesteuert wird und in dem die Sensowerte
// gespeichert wertden.
// value -> Sensorwerte
// Rückgabe der gemitteleten Sensorwerte
float getAvgerageSensor(int &iterator,float* arr, float value){
        // Null-Fehler abfangen
        if (value == 0) {
                return (arr[0] + arr[1] + arr[3]) / 3.0f;
        }
        if (iterator > 3) {
                iterator = 0;
        }
        arr[iterator] = value;
        iterator++;
        return (arr[0] + arr[1] + arr[3]) / 3.0f;
}

// Gibt ein True zurueck ,wenn die Sensorwerte eingependelt sind.
// Die Werte sind dann eingependelt, wenn sie sich weniger als
// 5cm von einander unterscheiden.
// arr -> Array in dem Sensorwerte gespeichert sind
bool areSensorValsSteady (float* arr){
    return  (fabs(arr[0] - arr[1]) < 0.05) &&
            (fabs(arr[1] - arr[2]) < 0.05) &&
            (fabs(arr[2] - arr[3]) < 0.05);
}

// Abfangen des Sprungs zwischen -180 und 180 Grad
// cur_yaw -> gemessener Yaw
// prev_yaw -> Yaw aus der letzten Messung
// relAngle -> Aufsummierter Yaw aus der Differenz cur_yaw und prev_yaw
// relAngle ist hierbei der Parameter mit dem weitergearbeitet wird. Die
// Funktion hat keinen Rüchgabewert sondern veraendert die Variable relAngle
// die als Referenz uebergeben wird, direkt.
void  yawCorrection (float &prev_yaw, float cur_yaw, float &relAngle){
        if (cur_yaw - prev_yaw > 5.0f) {
                relAngle += fabs(cur_yaw + prev_yaw);
        }
        else if (cur_yaw - prev_yaw < -5.0f) {
                relAngle -= fabs(cur_yaw + prev_yaw);

        } else {
                relAngle += cur_yaw - prev_yaw;
        }
        prev_yaw = cur_yaw;
}

// Berechnet aus dem Abstand zur Wand den Kurvenwinkel
// fuer das Rueckwaertsfahren
// a -> Wandabstand
// r -> Kurvenradius linkes Rad
// r0 -> Kruvenradius rechtes Rad
// gibt den Winkel als rad Wert zurueck
float getCurveAngle(float a, float r, float r0) {
  return (float) acos(1 - (a / (r + r0)));
}

// brechnet die Parklueckenlaenge abhaenging zum Wandabstand
// a -> Wandabstand
// r -> Kurvenradius linkes Rad
// r0 -> Kruvenradius rechtes Rad
// gibt die Laenge der Parkluecke zurueck
float getParkingSpaceLength (float a, float r , float r0){
    return sin(getCurveAngle(a,r,r0)) * (r + r0);
}

// Berechent den Winkel, wie schreag das Fahrzeug zur Wand steht.
// distA -> Wandabstand erster Messwert
// distE -> Wandabstand zu einem spaeteren Zeitpunkt
// strecke -> zurueck gelegete Strecke
// Gibt den Winkel als rad zurueck
float getDriftAngle (float distA, float distE, float strecke){
    return atan((distE-distA)/strecke);
}

//yaw aus Car-info
void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg, nav_msgs::Odometry* odom){
        *odom = *msg;
}

void sensorCallback(const pses_basis::SensorData::ConstPtr& sens_msg, pses_basis::SensorData *sensors){
        *sensors = *sens_msg;
}

void carInfoCallback(const pses_basis::CarInfo::ConstPtr& carInfo_msg, pses_basis::CarInfo *carInfo){
        *carInfo = *carInfo_msg;
}


int main(int argc, char **argv) {
        ros::init(argc, argv, "segfault_einparken");
        ros::NodeHandle npark;
        ros::Rate loop_rate(40);
/*Subscriptions from other ROS Nodes*/
//Initialize variables
        pses_basis::Command cmd;
        pses_basis::SensorData sensors;
        sensor_msgs::Imu imu;
        nav_msgs::Odometry odom;
        pses_basis::CarInfo carInfo;

        //Pubs and Subs
        ros::Publisher commands_pub = npark.advertise<pses_basis::Command>("pses_basis/command", 10);
        ros::Subscriber odom_sub = npark.subscribe<nav_msgs::Odometry>("odom",50, std::bind(odometryCallback, std::placeholders::_1, &odom));
        ros::Subscriber sensor_sub = npark.subscribe<pses_basis::SensorData>("pses_basis/sensor_data", 10, std::bind(sensorCallback, std::placeholders::_1, &sensors));
        ros::Subscriber carInfo_sub = npark.subscribe<pses_basis::CarInfo>("pses_basis/car_info",10, std::bind(carInfoCallback, std::placeholders::_1, &carInfo));

        ros::Time::init();

        // Korrekturwinkel waehrend dem Rueckwaertseinparken
        const float omega = 0.1526f;
        // Abstands korrektur gemessene werte
        const float offsetParken = 0.22f;
        const float minLenkradius = 0.6f;
        const float achsenLaenge = 0.154f;



        // -------- Statemachine -------------
		enum State {
			START,
       	 	RESET,
        	NEXT_TO_WALL,
        	NEXT_TO_OBSTICAL,
         	SET_PARAMETER,
         	BACKWARDS_LEFT,
          	BACKWARDS_RIGHT,
          	PARKED,
          	WAIT,
          	DELAY ,
          	OFFSET_DIST
		};

        State state = START;
        State nextstate = START;
        // ------ Statemachine ende -------------

  	 	double prev_time = 0;
        float wait_time = 0;

        int it = 0;
        float avgSensRight = 0;
        float sensorVals[3] = {1.0,1.0,1.0};

        float cur_angle = 0;
        float prev_yaw = 0;

        float lueckenlaenge = 0;
        float lueckenAnfang = 0;
        float drift = 0;
        float target_angle = 0;
        float refAbstand = 0;
        float refAbstandEnde = 0;
        float distanz = 0;
        float restWeg = 0;

        while(ros::ok()) {

            // gemittelte Sensorwerte
            avgSensRight = getAvgerageSensor(it,sensorVals,sensors.range_sensor_right);

            // Drehwinkel des Fahrzeugs
            yawCorrection(prev_yaw, carInfo.yaw, cur_angle);

            // Testen: Resetten der Statemachine in ihren Anfangszustand, wenn der Frontsensor
            // zugehalten wird
            if (sensors.range_sensor_front <= 0.1f && sensors.range_sensor_front != 0) {
                    cmd.motor_level = 0;
                    cmd.steering_level = 0;
                    state = START;
            }

            // Einparklogik
          	// 1. START -> WAIT -> RESET -> NEXT_TO_WALL -> PARKED
          	// 2.						   	  |-----> NEXT_TO_OBSTICAL -> DELAY -> NEXT_TO_WALL
          	// 3.						   	  |-----> OFFSET_DIST -> WAIT -> SET_PARAMETER -> BACKWARDS_RIGHT -> BACKWARDS_LEFT -> PARKED
            // 1. Auto steht zu nah an der Wand und parkt sofort
            // 2. Auto fährt an der Wand und regestriert ein Hindernis und springt wieder zurueck zur Luecken suche (NEXT_TO_WALL)
            // 3. Auto hat eine passende Luecke entdeckt und leitet das Parkmanoever ein

            switch (state) {
                case START:
                // Da im State START , WAIT aufgerufen wird, muessen mehrere Parameter
                // gespeichert werden, da diese Parameter im State "WAIT" erforderlich sind.
                // Zum einen die Anfangszeit prev_time, der nach Ablauf von Wait folgende State "nextstate".
                // und wait_time um die Wartezeit von WAIT festzulegen.
                    prev_time = ros::Time::now().toSec();
                    nextstate = RESET;
                    wait_time = 1.0f;
                    state = WAIT;
                    break;

                case RESET:
                    // Resetten aller Parameter
                    cmd.steering_level = -6;
                    cmd.motor_level = 7;
                    drift = 0;
                    refAbstand = avgSensRight;
                    refAbstandEnde = avgSensRight;
                    cur_angle = 0;
                  	lueckenAnfang = carInfo.driven_distance;
                    state = NEXT_TO_WALL;
                    break;

                case NEXT_TO_WALL:
                // Voraussetzung: Das Auto MUSS neben einer Wand stehen.
                // Eigentlicher Beginn der Parklogik. Das Auto sucht nun eine
                // Parkluecke.

                	// Falls das Auto zu nah an der Wand (20 cm) steht/startet, wird das Auto sofort
                	// als geparkt betrachtet
					if (refAbstand < 0.2f){
                    	state = PARKED;
                    	break;
                    }


                    if (areSensorValsSteady(sensorVals)){
                        refAbstandEnde = avgSensRight;
                    }
                    // Zureuckgelegte Distanz seit Eintritt in den Zustand NEXT_TO_WALL
                	distanz = carInfo.driven_distance - lueckenAnfang;
                	// Parklueckenlaenge wird in jedem neuen Takt angepasst
                	lueckenlaenge = getParkingSpaceLength(refAbstand, minLenkradius, minLenkradius + achsenLaenge) * 0.85;

                	// Objekt wurde an der Seite erkannt, unterbricht den Zustand und wechselt zu
                	// NEXT_TO_OBSTICAL
                	if (refAbstand - avgSensRight >= 0.1f){
                		state = NEXT_TO_OBSTICAL;
                		break;
                	}

                	// Falls keine der obigen if-Bedingungen eintreffen, wurde eine Luecke gefunden.
                	// Ansonsten wird NEXT_TO_WALL solange wiederholt, bis eine dieser Bedingungen eintrifft.
                	if (distanz >= lueckenlaenge ) {
                        drift = getDriftAngle (refAbstand, refAbstandEnde, distanz);
                        restWeg = carInfo.driven_distance;
                        state = OFFSET_DIST;
                	}
                	break;

                case OFFSET_DIST:
                	// Das Auto soll noch eine kleine Distanz weiter fahren damit es beim
                	// Einparken nicht an ein anderes Objekt stoesst, wenn es die Parkposition
                	// erreicht hat. Dann soll es stehen bleiben (die 1 sek Wait damit es ausrollen
                	// kann und tatsaechlich still steht)
                	if (carInfo.driven_distance - restWeg > 0.07f){
                		cmd.motor_level= 0 ;
                 		prev_time = ros::Time::now().toSec();
                    	nextstate = SET_PARAMETER;
                    	wait_time = 1.0f;
                    	state = WAIT;
                    }
                	break;

                case NEXT_TO_OBSTICAL:

                	// Das Auto befindet sich neben einem Objekt und faehrt nun solange
                	// geradeaus weiter bis es eine Kante bzw. die Wand wieder erkannt hat.
                    if ((refAbstand - avgSensRight) <= 0.1f) {
                        lueckenAnfang = carInfo.driven_distance;
                        state = DELAY;
                    }
                    break;

                case DELAY:

                    // Da wir mit gemittelten Sensordaten arbeiten, dauert es ein paar Takte bis
                    // sich die Sensorwerte korrekt, nach einer Kante, eingestellt haben. Dass ist nötig
                	// damit der neue refAbstand fehlerfrei gesetzt werden kann.
                    if (areSensorValsSteady(sensorVals)) {
                        refAbstand = avgSensRight;
                        state = NEXT_TO_WALL;
                    }
                    break;

                case SET_PARAMETER:
                	// Berechnet den notwendigen Winkel fuer die Kurvenfahrt
                    target_angle = getCurveAngle(refAbstandEnde - offsetParken , minLenkradius, minLenkradius + achsenLaenge);
					// Setzt Parameter füer die Kurvenfahrt
					cur_angle = 0;
                    cmd.motor_level = -15;
                    cmd.steering_level = -50;
                    state = BACKWARDS_RIGHT;
                    break;

                case WAIT:
					// WAIT : Wait loop wird solange ausgeführt, wie in dem vorherigen Zustand festgelegt
					// wurde und wechselt zu dem naechsten Zustand
                    if (ros::Time::now().toSec() - prev_time >= wait_time) {
                        state = nextstate;
                    }
                    break;

                case BACKWARDS_RIGHT:
                // Parkroutine: Faehrt eine Rechtskurve und danach eine LinksKurve

                    // geschwindigkeit reduzieren bei 75% der Kurvenfahrt
                    if ((fabs(cur_angle) + omega + drift) >= (target_angle * 0.75f )){
                        cmd.motor_level = -12;
                    }
                    // geschwindigkeit reduzieren bei 85% der Kurvenfahrt
                    if ((fabs(cur_angle) + omega + drift) >= (target_angle * 0.85f )){
                        cmd.motor_level = -9;
                    }
                    // wendepunkt erreicht, beschleunigen und nach links lenken
                    if ((fabs(cur_angle) + omega + drift) >= target_angle){
                        cur_angle = 0;
                        cmd.motor_level = -15;
                        cmd.steering_level = 50;
                        state = BACKWARDS_LEFT;
                    }
                    break;

                case BACKWARDS_LEFT:

                   // geschwindigkeit reduzieren bei 75% der Kurvenfahrt
                    if ((fabs(cur_angle) + omega) >= (target_angle * 0.75f )){
                        cmd.motor_level = -12;
                    }
                    // geschwindigkeit reduzieren bei 85% der Kurvenfahrt
                    if ((fabs(cur_angle) + omega) >= (target_angle * 0.85f )){
                        cmd.motor_level = -9;
                        // Wenn, Wand zu nah ist, stehen bleiben
                    	if (avgSensRight < 0.1f){
                       	 	state = PARKED;
                        	break;
                   		 }
                    }
                    // parkposition erreicht und anhalten
                    if ((fabs(cur_angle) + omega) >= target_angle){
                        cmd.motor_level = 0;
                        cmd.steering_level = 0;
                        state = PARKED;
                    }
                    break;

                case PARKED:
                    cmd.steering_level = 0;
                    cmd.motor_level = 0;
                    break;

                default:
					           cmd.steering_level = 0;
                     cmd.motor_level = 0;
                    break;
                }

                commands_pub.publish(cmd);
                ros::spinOnce();
                loop_rate.sleep();
        }
        ros::spin();
}
