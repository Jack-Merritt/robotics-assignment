
#include <Zumo32U4.h>
#include <limits.h>
#include <stack>
#include <queue>

Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;

const int THRESHOLD_VALUE = 1000;

const int MAZE_HEIGHT = 10;
const int MAZE_WIDTH = 10;

// Define robot's current position
int robotX = 0;
int robotY = 0;

// Define maze map
char mazeMap[MAZE_HEIGHT][MAZE_WIDTH];

// Define distance and visited arrays for Dijkstra's shortest path
int dist[MAZE_HEIGHT][MAZE_WIDTH];
bool visited[MAZE_HEIGHT][MAZE_WIDTH];

// Define directions for movement
const int dx[] = {0, 1, 0, -1}; // Right, Down, Left, Up
const int dy[] = {1, 0, -1, 0};

// Define parent array for path reconstruction
pair<int, int> parent[MAZE_HEIGHT][MAZE_WIDTH];

void setup() {
  lineSensors.initFiveSensors();
  initializeMap();
  dijkstra(); // Calculate shortest paths after initializing the map
}

void loop() {
  unsigned int sensorValues[5];
  lineSensors.read(sensorValues);


  if (areAllSensorsOnWhite(sensorValues)) {
    moveForward();
    updateMap();
  } else {
    // If any sensor detects black, stop and check for dead end or potential house
    stopMotors();
    delay(100);
    if (isDeadEnd(sensorValues)) {
      
      turnAround();
      updateMap();
    } else if (isPotentialHouse(sensorValues)) {
     
      approachHouse();
      updateMap();
    } else {
      
      turnLeft();
      updateMap();
    }
    delay(500);
  }
}

bool areAllSensorsOnWhite(unsigned int *sensorValues) {
  for (int i = 0; i < 5; i++) {
    if (sensorValues[i] < THRESHOLD_VALUE) {
      return false;
    }
  }
  return true;
}

bool isDeadEnd(unsigned int *sensorValues) {

  for (int i = 0; i < 5; i++) {
    if (sensorValues[i] >= THRESHOLD_VALUE) {
      return false; 
    }
  }
  return true;
}

bool isPotentialHouse(unsigned int *sensorValues) {
  

  


  int zumoWidth = 10;

  // Calculate the dimensions of the enclosed area
  int enclosedWidth = 5 * zumoWidth;
  int entryWidth = 2.5 * zumoWidth;

  for (int i = 0; i < 5; i++) {
    if (sensorValues[i] >= entryWidth) {
      return true;
    }
  }
  return false;
}

void moveForward() {
  motors.setSpeeds(200, 200);
}

void stopMotors() {
  motors.setSpeeds(0, 0);
}

void turnLeft() {
  motors.setSpeeds(-200, 200);
}

void turnAround() {
  motors.setSpeeds(-200, 200); 
  delay(500); 
}

void approachHouse() {
  const int SLOW_SPEED = 50; 

  /
  motors.setSpeeds(SLOW_SPEED, SLOW_SPEED);
  delay(2000); 
  motors.setSpeeds(0, 0); 
  returnHome();

}

void updateMap() {
  // Update the maze map based on the robot's movements
  mazeMap[robotY][robotX] = 'X'; // Mark current position as visited
}

void initializeMap() {
  // Initialize the maze map with default values
  for (int i = 0; i < MAZE_HEIGHT; i++) {
    for (int j = 0; j < MAZE_WIDTH; j++) {
      mazeMap[i][j] = ' '; // Empty space
      dist[i][j] = INT_MAX; // Initialize distances to maximum value
      visited[i][j] = false; // Initialize all cells as unvisited
    }
  }
  // Mark the starting position in the maze map
  mazeMap[robotY][robotX] = 'S'; // 'S' for start
  dist[robotY][robotX] = 0; // Distance to starting position is 0
}

void dijkstra() {
  // Implementation of shortest path
  // store cells
  priority_queue<pair<int, pair<int, int>>, vector<pair<int, pair<int, int>>>, greater<pair<int, pair<int, int>>>> pq;

  // Add starting position to priority queue
  pq.push({0, {robotX, robotY}});
 
  parent[robotX][robotY] = {robotX, robotY};

  while (!pq.empty()) {
  
    int u = pq.top().second.first;
    int v = pq.top().second.second;
    int w = pq.top().first;
    pq.pop();

    
    visited[u][v] = true;

    // Explore all adjacent cells
    for (int i = 0; i < 4; i++) {
      int newX = u + dx[i];
      int newY = v + dy[i];

      
      if (isValid(newX, newY) && !visited[newX][newY]) {
        // Calculate new distance
        int newDist = w + 1; // Assuming unit weight for each cell

        // Update distance if new distance is shorter
        if (newDist < dist[newX][newY]) {
          dist[newX][newY] = newDist;
          pq.push({newDist, {newX, newY}});
          // Update parent
          parent[newX][newY] = {u, v};
        }
      }
    }
  }
}

bool isValid(int x, int y) {
  // Check if position (x, y) is within maze bounds
  return (x >= 0 && x < MAZE_HEIGHT && y >= 0 && y < MAZE_WIDTH);
}

void reconstructPath(pair<int, int> source, pair<int, int> destination) {
  // Reconstruct the shortest path using the parent array
  stack<pair<int, int>> path;
  pair<int, int> current = destination;

  // Trace back the path from destination to source
  while (current != source) {
    path.push(current);
    current = parent[current.first][current.second];
  }
  path.push(source); // Push the source cell
 // Print or use the path
  while (!path.empty()) {
    pair<int, int> cell = path.top();
    path.pop();
    Serial.print("->(");
    Serial.print(cell.first);
    Serial.print(",");
    Serial.print(cell.second);
    Serial.print(") ");
  }
}
