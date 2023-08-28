// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import { NT4_Client } from "./NT4.js";

const middleOfBotRobotToDashboardTopic = "/nodeselector/middleOfBot_robot_to_dashboard";
const middleOfBotDashboardToRobotTopic = "/nodeselector/middleOfBot_dashboard_to_robot";
const highIntakeRobotToDashboardTopic = "/nodeselector/highIntake_robot_to_dashboard";
const highIntakeDashboardToRobotTopic = "/nodeselector/highIntake_dashboard_to_robot";
const groundRobotToDashboardTopic = "/nodeselector/ground_robot_to_dashboard";
const groundDoubleClick = "/nodeselector/ground_double_click";
const groundDashboardToRobotTopic = "/nodeselector/ground_dashboard_to_robot";
const nodeRobotToDashboardTopic = "/nodeselector/node_robot_to_dashboard";
const nodeDashboardToRobotTopic = "/nodeselector/node_dashboard_to_robot";
const matchTimeTopic = "/nodeselector/match_time";
const isAutoTopic = "/nodeselector/is_auto";

let active = null;
let matchTime = 0;
let isAuto = false;
let middleOfBot = false;
let highIntake = false;
let ground = false;

function displayActive(index) {
  active = index;
  Array.from(document.getElementsByClassName("active")).forEach((element) => {
    element.classList.remove("active");
  });
  if (index !== null) {
    document.getElementsByTagName("td")[index].classList.add("active");
  }
}

function sendActive(index) {
  if (index !== active) {
    client.addSample(nodeDashboardToRobotTopic, index);
  }
}

function displaymiddleOfBot(newmiddleOfBot) {
  middleOfBot = newmiddleOfBot;
}
function sendMiddleOfBot() {
  client.addSample(middleOfBotDashboardToRobotTopic, true);
}

function displayhighIntake(newhighIntake) {
  highIntake = newhighIntake;
}
function sendHighIntake() {
  client.addSample(highIntakeDashboardToRobotTopic, true);
}

function displayGround(newGround) {
  ground = newGround;
}
function sendGround() {
  client.addSample(groundDashboardToRobotTopic, true);
}

function sendGroundDoubleClick() {
  client.addSample(groundDoubleClick, true);
}

function displayTime(time, isAuto) {
  let element = document.getElementsByClassName("time")[0];
  element.className = "time";
  if (isAuto) {
    element.classList.add("auto");
  } else if (time > 41 || time == 0) {
    element.classList.add("teleop-1");
  } else if (time > 15) {
    element.classList.add("teleop-2");
  } else {
    element.classList.add("teleop-3");
  }
  let secondsString = (time % 60).toString();
  if (secondsString.length == 1) {
    secondsString = "0" + secondsString;
  }
  element.innerText = Math.floor(time / 60).toString() + ":" + secondsString;
}


let client = new NT4_Client(
  window.location.hostname,
  "NodeSelector",
  (topic) => {
    // Topic announce
  },
  (topic) => {
    // Topic unannounce
  },
  (topic, timestamp, value) => {
    // New data
    if (topic.name === nodeRobotToDashboardTopic) {
      document.body.style.backgroundColor = "white";
      displayActive(value);
    } else if (topic.name === middleOfBotRobotToDashboardTopic) {
      displaymiddleOfBot(value);
    } else if (topic.name === highIntakeRobotToDashboardTopic) {
      displayhighIntake(value);
    } else if (topic.name === groundRobotToDashboardTopic) {
      displayGround(value);
    }else if (topic.name === matchTimeTopic) {
      matchTime = value;
      displayTime(matchTime, isAuto);
    } else if (topic.name === isAutoTopic) {
      isAuto = value;
      displayTime(matchTime, isAuto);
    }
  },
  () => {
    // Connected
  },
  () => {
    // Disconnected
    document.body.style.backgroundColor = "red";
    displayActive(null);
    displayTime(0, false);
    displaymiddleOfBot(false);
    displayhighIntake(false);
    displayGround(false);
  }
);

window.addEventListener("load", () => {
  // Start NT connection
  client.subscribe(
    [
      nodeRobotToDashboardTopic,
      middleOfBotRobotToDashboardTopic,
      highIntakeRobotToDashboardTopic,
      groundRobotToDashboardTopic,
      matchTimeTopic,
      isAutoTopic,
    ],
    false,
    false,
    0.02
  );
  client.publishTopic(nodeDashboardToRobotTopic, "int");
  client.publishTopic(middleOfBotDashboardToRobotTopic, "boolean");
  client.publishTopic(highIntakeDashboardToRobotTopic, "boolean");
  client.publishTopic(groundDashboardToRobotTopic, "boolean");
  client.publishTopic(groundDoubleClick, "boolean");
  client.connect();

  // Add node click listeners
  Array.from(document.getElementsByClassName("node")).forEach((cell, index) => {
    cell.addEventListener("click", () => {
      sendActive(index);
    });
    cell.addEventListener("contextmenu", (event) => {
      event.preventDefault();
      sendActive(index);
    });
  });

  // Add node touch listeners
  [("touchstart", "touchmove")].forEach((eventString) => {
    document.body.addEventListener(eventString, (event) => {
      if (event.touches.length > 0) {
        let x = event.touches[0].clientX;
        let y = event.touches[0].clientY;
        Array.from(document.getElementsByClassName("node")).forEach(
          (cell, index) => {
            let rect = cell.getBoundingClientRect();
            if (
              x >= rect.left &&
              x <= rect.right &&
              y >= rect.top &&
              y <= rect.bottom
            ) {
              sendActive(index);
            }
          }
        );
      }
    });
  });

  // Add middleOfBot listeners
  const middleOfBotClickedDiv =
    document.getElementsByClassName("middleOfBot")[0];
  middleOfBotClickedDiv.addEventListener("click", () => {
    sendMiddleOfBot();
  });
  middleOfBotClickedDiv.addEventListener("contextmenu", (event) => {
    event.preventDefault();
    sendMiddleOfBot();
  });
  middleOfBotClickedDiv.addEventListener("touchstart", () => {
    event.preventDefault();
    sendMiddleOfBot();
  });

  // Add HighIntake listeners
  const highIntakeClickedDiv =
    document.getElementsByClassName("highIntake")[0];
  highIntakeClickedDiv.addEventListener("click", () => {
    sendHighIntake();
  });
  highIntakeClickedDiv.addEventListener("contextmenu", (event) => {
    event.preventDefault();
    sendHighIntake();
  });
  highIntakeClickedDiv.addEventListener("touchstart", () => {
    event.preventDefault();
    sendHighIntake();
  });
  
  // Add ground listeners
  const groundClickedDiv = document.getElementsByClassName("ground")[0];

  groundClickedDiv.addEventListener("click", () => {
    sendGround();
  });
  groundClickedDiv.addEventListener("contextmenu", (event) => {
    event.preventDefault();
    sendGround();
  });
  groundClickedDiv.addEventListener("touchstart", (event) => {
    event.preventDefault();
    sendGround();
  });

  groundClickedDiv.addEventListener("dblclick", () => {
    event.preventDefault();
    sendGroundDoubleClick();
  });

});
