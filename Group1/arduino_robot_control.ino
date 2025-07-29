#include <Servo.h>

Servo joint1, joint2, joint3, joint4, gripper;

int angle1=90, angle2=90, angle3=90, angle4=90, gripperAngle=90;

void setup() {
  Serial.begin(9600);
  joint1.attach(3);
  joint2.attach(5);
  joint3.attach(6);
  joint4.attach(9);
  gripper.attach(10);
  updateAll();
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "J1+") angle1 += 5;
    else if (cmd == "J1-") angle1 -= 5;
    else if (cmd == "J2+") angle2 += 5;
    else if (cmd == "J2-") angle2 -= 5;
    else if (cmd == "J3+") angle3 += 5;
    else if (cmd == "J3-") angle3 -= 5;
    else if (cmd == "J4+") angle4 += 5;
    else if (cmd == "J4-") angle4 -= 5;
    else if (cmd == "GRIPPER=ON") gripperAngle = 40;
    else if (cmd == "GRIPPER=OFF") gripperAngle = 90;
    else if (cmd == "POS+X") {/* logic */}
    else if (cmd == "POS-X") {/* logic */}
    else if (cmd == "POS+Z") {/* logic */}
    else if (cmd == "POS-Z") {/* logic */}
    else if (cmd == "POS=HOME" || cmd == "CMD=HOME") {
      angle1=angle2=angle3=angle4=gripperAngle=90;
    }
    else if (cmd == "CMD=SETTING") {
      // logic setting nếu cần
    }

    constrainAngles();
    updateAll();
    sendTMatrix();
  }
}

void updateAll() {
  joint1.write(angle1);
  joint2.write(angle2);
  joint3.write(angle3);
  joint4.write(angle4);
  gripper.write(gripperAngle);
}

void constrainAngles() {
  angle1 = constrain(angle1, 0, 180);
  angle2 = constrain(angle2, 0, 180);
  angle3 = constrain(angle3, 0, 180);
  angle4 = constrain(angle4, 0, 180);
  gripperAngle = constrain(gripperAngle, 0, 180);
}

void sendTMatrix() {
  Serial.println("TMATRIX:1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1");
}