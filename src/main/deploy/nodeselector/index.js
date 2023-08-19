// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import { NT4_Client } from "./NT4.js";

const nodeRobotToDashboardTopic = "/nodeselector/node_robot_to_dashboard";
const nodeDashboardToRobotTopic = "/nodeselector/node_dashboard_to_robot";
const matchTimeTopic = "/nodeselector/match_time";
const isAutoTopic = "/nodeselector/is_auto";

let active = null;
let matchTime = 0;
let isAuto = false;

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
    } else if (topic.name === matchTimeTopic) {
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
  }
);

window.addEventListener("load", () => {
  // Start NT connection
  client.subscribe(
    [
      nodeRobotToDashboardTopic,
      matchTimeTopic,
      isAutoTopic,
    ],
    false,
    false,
    0.02
  );
  client.publishTopic(nodeDashboardToRobotTopic, "int");
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
});
