import controlP5.*;  // import ControlP5 library
import processing.serial.*;

Serial port;

ControlP5 cp5;  // create 
PFont font, font2;
Textfield m11, m12, m13, m14, m21, m22, m23, m24, m31, m32, m33, m34, m41, m42, m43, m44, path, posX, posY, goalX, goalY, irF, irR, irB, irL;
boolean sent = false;
int[] data = new int[22];
int dataIndex = 0;
String[] outputData = new String[22];

void setup() {

  size(300, 480); // window size, (width, height)

  port = new Serial(this, "COM6", 9600);  // Connect to uno

  //buttons

  cp5 = new ControlP5(this);
  font = createFont("calibri light", 20);
  font2 = createFont("calibri light", 12);

  cp5.addButton("SendData")
    .setPosition(80, 30)
    .setSize(120, 40)
    .setFont(font)
    ;
  m11 = cp5.addTextfield("m11")
    .setPosition(20, 80)
    .setSize(40, 40)
    .setFont(font)
    ;  
  m12 = cp5.addTextfield("m12")
    .setPosition(60, 80)
    .setSize(40, 40)
    .setFont(font)
    ;  
  m13 = cp5.addTextfield("m13")
    .setPosition(100, 80)
    .setSize(40, 40)
    .setFont(font)
    ;  
  m14 = cp5.addTextfield("m14")
    .setPosition(140, 80)
    .setSize(40, 40)
    .setFont(font)
    ;
  m21 = cp5.addTextfield("m21")
    .setPosition(20, 120)
    .setSize(40, 40)
    .setFont(font)
    ;  
  m22 = cp5.addTextfield("m22")
    .setPosition(60, 120)
    .setSize(40, 40)
    .setFont(font)
    ;  
  m23 = cp5.addTextfield("m23")
    .setPosition(100, 120)
    .setSize(40, 40)
    .setFont(font)
    ;  
  m24 = cp5.addTextfield("m24")
    .setPosition(140, 120)
    .setSize(40, 40)
    .setFont(font)
    ;   
  m31 = cp5.addTextfield("m31")
    .setPosition(20, 160)
    .setSize(40, 40)
    .setFont(font)
    ;  
  m32 = cp5.addTextfield("m32")
    .setPosition(60, 160)
    .setSize(40, 40)
    .setFont(font)
    ;  
  m33 = cp5.addTextfield("m33")
    .setPosition(100, 160)
    .setSize(40, 40)
    .setFont(font)
    ;  
  m34 = cp5.addTextfield("m34")
    .setPosition(140, 160)
    .setSize(40, 40)
    .setFont(font)
    ;   
  m41 = cp5.addTextfield("m41")
    .setPosition(20, 200)
    .setSize(40, 40)
    .setFont(font)
    ;  
  m42 = cp5.addTextfield("m42")
    .setPosition(60, 200)
    .setSize(40, 40)
    .setFont(font)
    ;  
  m43 = cp5.addTextfield("m43")
    .setPosition(100, 200)
    .setSize(40, 40)
    .setFont(font)
    ;  
  m44 = cp5.addTextfield("m44")
    .setPosition(140, 200)
    .setSize(40, 40)
    .setFont(font)
    ;  
  path = cp5.addTextfield("path")
    .setPosition(200, 80)
    .setSize(80, 40)
    .setFont(font2)
    ;  
  posX = cp5.addTextfield("posX")
    .setPosition(200, 140)
    .setSize(40, 40)
    .setFont(font2)
    ;  
  posY = cp5.addTextfield("posY")
    .setPosition(240, 140)
    .setSize(40, 40)
    .setFont(font2)
    ; 
  goalX = cp5.addTextfield("goalX")
    .setPosition(200, 200)
    .setSize(40, 40)
    .setFont(font2)
    ;  
  goalY = cp5.addTextfield("goalY")
    .setPosition(240, 200)
    .setSize(40, 40)
    .setFont(font2)
    ;  
  irF = cp5.addTextfield("irF")
    .setPosition(120, 260)
    .setSize(60, 60)
    .setFont(font)
    ;  
  irR = cp5.addTextfield("irR")
    .setPosition(180, 320)
    .setSize(60, 60)
    .setFont(font)
    ;  
  irB = cp5.addTextfield("irB")
    .setPosition(120, 380)
    .setSize(60, 60)
    .setFont(font)
    ;  
  irL = cp5.addTextfield("irL")
    .setPosition(60, 320)
    .setSize(60, 60)
    .setFont(font)
    ;

  m11.getCaptionLabel().setVisible(false);
  m12.getCaptionLabel().setVisible(false);
  m13.getCaptionLabel().setVisible(false);
  m14.getCaptionLabel().setVisible(false);
  m21.getCaptionLabel().setVisible(false);
  m22.getCaptionLabel().setVisible(false);
  m23.getCaptionLabel().setVisible(false);
  m24.getCaptionLabel().setVisible(false);
  m31.getCaptionLabel().setVisible(false);
  m32.getCaptionLabel().setVisible(false);
  m33.getCaptionLabel().setVisible(false);
  m34.getCaptionLabel().setVisible(false);
  m41.getCaptionLabel().setVisible(false);
  m42.getCaptionLabel().setVisible(false);
  m43.getCaptionLabel().setVisible(false);
  m44.getCaptionLabel().setVisible(false);
  irF.getCaptionLabel().setVisible(false);
  irR.getCaptionLabel().setVisible(false);
  irB.getCaptionLabel().setVisible(false);
  irL.getCaptionLabel().setVisible(false);
}

void draw() {  // loop
  background(150, 0, 150);  // background color of window (rgb) (up to 255)

  if (sent) {
    ReadData();
  }
  if (dataIndex > 19) {
    WriteData();
  }
}

// button functions

void SendData() {
  if (!sent) {
    port.write(path.getText() + "\n");
    port.write(98 + "\n");
    port.write(posX.getText() + "\n");
    port.write(posY.getText() + "\n");
    port.write(97 + "\n");
    port.write(goalX.getText() + "\n");
    port.write(goalY.getText() + "\n");
    port.write(98 + "\n");
    port.write(m11.getText() + "\n");
    port.write(m12.getText() + "\n");
    port.write(m13.getText() + "\n");
    port.write(m14.getText() + "\n");
    port.write(m21.getText() + "\n");
    port.write(m22.getText() + "\n");
    port.write(m23.getText() + "\n");
    port.write(m24.getText() + "\n");
    port.write(m31.getText() + "\n");
    port.write(m32.getText() + "\n");
    port.write(m33.getText() + "\n");
    port.write(m34.getText() + "\n");
    port.write(m41.getText() + "\n");
    port.write(m42.getText() + "\n");
    port.write(m43.getText() + "\n");
    port.write(m44.getText() + "\n");
    port.write(99 + "\n");
    sent = true;
  }
}

void ReadData() {
  while (port.available() > 0) {
    outputData[dataIndex] = port.readStringUntil('\n'); 
    println(outputData[dataIndex] + " | " + dataIndex);
    if (outputData[dataIndex] != null) {
      dataIndex++;
    }
  }
}

void WriteData() {
  m11.setText(outputData[0]);
  m12.setText(outputData[1]);
  m13.setText(outputData[2]);
  m14.setText(outputData[3]);
  m21.setText(outputData[4]);
  m22.setText(outputData[5]);
  m23.setText(outputData[6]);
  m24.setText(outputData[7]);
  m31.setText(outputData[8]);
  m32.setText(outputData[9]);
  m33.setText(outputData[10]);
  m34.setText(outputData[11]);
  m41.setText(outputData[12]);
  m42.setText(outputData[13]);
  m43.setText(outputData[14]);
  m44.setText(outputData[15]);
  irF.setText(outputData[16]);
  irR.setText(outputData[17]);
  irB.setText(outputData[18]);
  irL.setText(outputData[19]);
  dataIndex = 0;
  java.util.Arrays.fill(outputData, "");
}
