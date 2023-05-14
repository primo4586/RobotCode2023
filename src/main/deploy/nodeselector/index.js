// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import { NT4_Client } from "./NT4.js";

const nodeRobotToDashboardTopic = "/nodeselector/node_robot_to_dashboard";
const nodeDashboardToRobotTopic = "/nodeselector/node_dashboard_to_robot";
const coneTippedRobotToDashboardTopic =
  "/nodeselector/cone_tipped_robot_to_dashboard";
const coneTippedDashboardToRobotTopic =
  "/nodeselector/cone_tipped_dashboard_to_robot";
const matchTimeTopic = "/nodeselector/match_time";
const isAutoTopic = "/nodeselector/is_auto";

let active = null;
let tipped = false;
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

function displayTipped(newTipped) {
  if (newTipped != tipped) {
    tipped = newTipped;
    var coneOrientation = document.getElementsByClassName("cone-orientation")[0];
    document
      .getElementsByClassName("cone-orientation")[0]
      .classList.toggle("tipped");
    var svgPath1 = coneOrientation.getElementsByTagName("path")[0];
    var svgPath2 = coneOrientation.getElementsByTagName("path")[1];
      
    if (coneOrientation.classList.contains("tipped")) {
      svgPath1.setAttribute(
        "d",
        "M11 6a3 3 0 1 1-6 0 3 3 0 0 1 6 0z"
      ); // New path when tipped
      svgPath2.setAttribute(
        "d",
        "M0 8a8 8 0 1 1 16 0A8 8 0 0 1 0 8zm8-7a7 7 0 0 0-5.468 11.37C3.242 11.226 4.805 10 8 10s4.757 1.225 5.468 2.37A7 7 0 0 0 8 1z"
      )
    } else {
      svgPath1.setAttribute(
        "d",
        "M8.5 1.866a1 1 0 1 0-1 0V3h-2A4.5 4.5 0 0 0 1 7.5V8a1 1 0 0 0-1 1v2a1 1 0 0 0 1 1v1a2 2 0 0 0 2 2h10a2 2 0 0 0 2-2v-1a1 1 0 0 0 1-1V9a1 1 0 0 0-1-1v-.5A4.5 4.5 0 0 0 10.5 3h-2V1.866ZM14 7.5V13a1 1 0 0 1-1 1H3a1 1 0 0 1-1-1V7.5A3.5 3.5 0 0 1 5.5 4h5A3.5 3.5 0 0 1 14 7.5Z"
      );
        svgPath2.setAttribute(
          "d",
          "M6 12.5a.5.5 0 0 1 .5-.5h3a.5.5 0 0 1 0 1h-3a.5.5 0 0 1-.5-.5ZM3 8.062C3 6.76 4.235 5.765 5.53 5.886a26.58 26.58 0 0 0 4.94 0C11.765 5.765 13 6.76 13 8.062v1.157a.933.933 0 0 1-.765.935c-.845.147-2.34.346-4.235.346-1.895 0-3.39-.2-4.235-.346A.933.933 0 0 1 3 9.219V8.062Zm4.542-.827a.25.25 0 0 0-.217.068l-.92.9a24.767 24.767 0 0 1-1.871-.183.25.25 0 0 0-.068.495c.55.076 1.232.149 2.02.193a.25.25 0 0 0 .189-.071l.754-.736.847 1.71a.25.25 0 0 0 .404.062l.932-.97a25.286 25.286 0 0 0 1.922-.188.25.25 0 0 0-.068-.495c-.538.074-1.207.145-1.98.189a.25.25 0 0 0-.166.076l-.754.785-.842-1.7a.25.25 0 0 0-.182-.135Z"
        );
         // Original path when not tipped
    }
  }
}

function displayTime(time, isAuto) {
  let element = document.getElementsByClassName("time")[0];
  element.className = "time";
  if (isAuto) {
    element.classList.add("auto");
  } else if (time > 30 || time == 0) {
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

function toggleTipped() {
  client.addSample(coneTippedDashboardToRobotTopic, !tipped);
  const coneOrientation = document.querySelector('.cone-orientation');
const svgElement = document.getElementById('svgElement');
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
    } else if (topic.name === coneTippedRobotToDashboardTopic) {
      displayTipped(value);
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
    displayTipped(false);
    displayTime(0, false);
  }
);

window.addEventListener("load", () => {
  // Start NT connection
  client.subscribe(
    [
      nodeRobotToDashboardTopic,
      coneTippedRobotToDashboardTopic,
      matchTimeTopic,
      isAutoTopic,
    ],
    false,
    false,
    0.02
  );
  client.publishTopic(nodeDashboardToRobotTopic, "int");
  client.publishTopic(coneTippedDashboardToRobotTopic, "boolean");
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

  // Add cone orientation listeners
  const coneOrientationDiv =
    document.getElementsByClassName("cone-orientation")[0];
  coneOrientationDiv.addEventListener("click", () => {
    toggleTipped();
  });
  coneOrientationDiv.addEventListener("contextmenu", (event) => {
    event.preventDefault();
    toggleTipped();
  });
  coneOrientationDiv.addEventListener("touchstart", () => {
    event.preventDefault();
    toggleTipped();
  });
});
