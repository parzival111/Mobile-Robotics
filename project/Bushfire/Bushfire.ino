
#include <TimerOne.h>       //include library for timers with interrupts

#define timer_rate 1                  // sensor update calls per second
#define timer_int 1000000/timer_rate   // timer interrupt interval in microseconds


uint8_t mapData[4][4];
uint8_t bushMap[4][4] =  { {16, 16, 16, 16},
  {16, 16, 16, 16},
  {16, 16, 16, 16},
  {16, 16, 16, 16}
};

uint8_t path[9];
uint8_t Position[2] = {0, 0};
uint8_t Goal[2] = {0, 0};

int counter = 0;
int pathIndex = 0;
int pathValue;
int heading = 1;
int nextHeading;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600); // open the serial port at 9600 baud


  //map 1
  mapData[0][0] = 16;
  mapData[0][1] = 2;
  mapData[0][2] = 8;
  mapData[0][3] = 12;
  mapData[1][0] = 10;
  mapData[1][1] = 7;
  mapData[1][2] = 10;
  mapData[1][3] = 7;
  mapData[2][0] = 9;
  mapData[2][1] = 6;
  mapData[2][2] = 3;
  mapData[2][3] = 12;
  mapData[3][0] = 15;
  mapData[3][1] = 14;
  mapData[3][2] = 5;
  mapData[3][3] = 7;

  // map 2
  /*
    mapData[0][0] = 12;
    mapData[0][1] = 16;
    mapData[0][2] = 16;
    mapData[0][3] = 12;
    mapData[1][0] = 9;
    mapData[1][1] = 6;
    mapData[1][2] = 6;
    mapData[1][3] = 3;
    mapData[2][0] = 11;
    mapData[2][1] = 16;
    mapData[2][2] = 16;
    mapData[2][3] = 11;
    mapData[3][0] = 13;
    mapData[3][1] = 8;
    mapData[3][2] = 16;
    mapData[3][3] = 15;
  */

  Position[0] = 3;
  Position[1] = 0;

  Goal[0] = 0;
  Goal[1] = 3;

  bushMap[Goal[0]][Goal[1]] = counter;

  for (int i = 0; i < 11; i++) {
    expand();
  }

  pathValue = bushMap[Position[0]][Position[1]];
  for (int i = pathValue; i > 0; i--) {
    makePath();
  }

  for (int i = 0; i < sizeof(path); i++) {
    Serial.println(path[i]);
  }
}

void loop() {


}

void displayMap() {
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      Serial.print("   ");
      Serial.print(mapData[row][col]);
      if (row == Position[0] && col == Position[1]) {
        Serial.print("P  ");
      }
      else if (row == Goal[0] && col == Goal[1]) {
        Serial.print("G  ");
      } else {
        Serial.print("   ");
      }
    }
    Serial.println();
  }
  Serial.println("\n\n\n");
}

void displayBushMap() {
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      Serial.print("   ");
      Serial.print(bushMap[row][col]);
      Serial.print("   ");
    }
    Serial.println();
  }
  Serial.println("\n\n\n");
}

void expand() {
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      if (bushMap[row][col] == counter) {
        switch (mapData[row][col]) {
          case 1:
            topOpen(row, col);
            rightOpen(row, col);
            bottomOpen(row, col);
            leftOpen(row, col);
            break;
          case 2:
            rightOpen(row, col);
            bottomOpen(row, col);
            leftOpen(row, col);
            break;
          case 3:
            topOpen(row, col);
            bottomOpen(row, col);
            leftOpen(row, col);
            break;
          case 4:
            bottomOpen(row, col);
            leftOpen(row, col);
            break;
          case 5:
            topOpen(row, col);
            rightOpen(row, col);
            leftOpen(row, col);
            break;
          case 6:
            rightOpen(row, col);
            leftOpen(row, col);
            break;
          case 7:
            topOpen(row, col);
            leftOpen(row, col);
            break;
          case 8:
            leftOpen(row, col);
            break;
          case 9:
            topOpen(row, col);
            rightOpen(row, col);
            bottomOpen(row, col);
            break;
          case 10:
            rightOpen(row, col);
            bottomOpen(row, col);
            break;
          case 11:
            topOpen(row, col);
            bottomOpen(row, col);
            break;
          case 12:
            bottomOpen(row, col);
            break;
          case 13:
            topOpen(row, col);
            rightOpen(row, col);
            break;
          case 14:
            rightOpen(row, col);
            break;
          case 15:
            topOpen(row, col);
            break;
          case 16:

            break;
          default:

            break;

        }

      }

    }

  }
  counter++;
  displayMap();
  displayBushMap();
}


void makePath() {
  int row = Position[0];
  int col = Position[1];
  switch (mapData[row][col]) {
    case 1:
      checktop(row, col);
      checkright(row, col);
      checkbottom(row, col);
      checkleft(row, col);
      break;
    case 2:
      checkright(row, col);
      checkbottom(row, col);
      checkleft(row, col);
      break;
    case 3:
      checktop(row, col);
      checkbottom(row, col);
      checkleft(row, col);
      break;
    case 4:
      checkbottom(row, col);
      checkleft(row, col);
      break;
    case 5:
      checktop(row, col);
      checkright(row, col);
      checkleft(row, col);
      break;
    case 6:
      checkright(row, col);
      checkleft(row, col);
      break;
    case 7:
      checktop(row, col);
      checkleft(row, col);
      break;
    case 8:
      checkleft(row, col);
      break;
    case 9:
      checktop(row, col);
      checkright(row, col);
      checkbottom(row, col);
      break;
    case 10:
      checkright(row, col);
      checkbottom(row, col);
      break;
    case 11:
      checktop(row, col);
      checkbottom(row, col);
      break;
    case 12:
      checkbottom(row, col);
      break;
    case 13:
      checktop(row, col);
      checkright(row, col);
      break;
    case 14:
      checkright(row, col);
      break;
    case 15:
      checktop(row, col);
      break;
    case 16:

      break;
    default:

      break;
  }
}




void topOpen(int row, int col) {
  if (row != 0) {
    if (bushMap[row - 1][col] == 16) {
      bushMap[row - 1][col] = counter + 1;
    }
  }
}

void rightOpen(int row, int col) {
  if (col != 3) {
    if (bushMap[row][col + 1] == 16) {
      bushMap[row][col + 1] = counter + 1;
    }
  }
}

void bottomOpen(int row, int col) {
  if (row != 3) {
    if (bushMap[row + 1][col] == 16) {
      bushMap[row + 1][col] = counter + 1;
    }
  }
}

void leftOpen(int row, int col) {
  if (row != 0) {
    if (bushMap[row][col - 1] == 16) {
      bushMap[row][col - 1] = counter + 1;
    }
  }
}


void checktop(int row, int col) {
  if (row != 0) {
    if (bushMap[row - 1][col] == pathValue - 1) {
      pathValue--;
      nextHeading = 1;
      calculateTurn();
    }
  }
}

void checkright(int row, int col) {
  if (col != 0) {
    if (bushMap[row][col + 1] == pathValue - 1) {
      Serial.println("Checking Right!");
      pathValue--;
      nextHeading = 2;
      calculateTurn();
    }
  }
}

void checkleft(int row, int col) {
  if (row != 3) {
    if (bushMap[row + 1][col] == pathValue - 1) {
      pathValue--;
      nextHeading = 3;
      calculateTurn();
    }
  }
}

void checkbottom(int row, int col) {
  if (col != 3) {
    if (bushMap[row][col - 1] == pathValue - 1) {
      pathValue--;
      nextHeading = 4;
      calculateTurn();
    }
  }
}

void calculateTurn() {
  if (nextHeading == heading + 1 || heading == 4 && nextHeading == 1) {
    path[pathIndex] = 3;
  }
  else if (nextHeading == heading - 1 || nextHeading == 4 && heading == 1) {
    path[pathIndex] = 1;
  }
  else {
    path[pathIndex] = 2;
  }
  pathIndex++;
  updatePosition();
  heading = nextHeading;
}

void updatePosition() {
  if (heading == 1) {
    Position[0]--;
  }
  if (heading == 2) {
    Position[1]--;
  }
  if (heading == 3) {
    Position[0]++;
  }
  if (heading == 4) {
    Position[1]--;
  }

}
