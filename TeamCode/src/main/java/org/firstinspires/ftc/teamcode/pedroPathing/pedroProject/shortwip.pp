{
  "startPoint": {
    "x": 22.811428571428557,
    "y": 127.58857142857143,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-uqqf6khhuz",
      "name": "Path 1",
      "endPoint": {
        "x": 53.8,
        "y": 80.16000000000004,
        "heading": "tangential",
        "startDeg": 143,
        "endDeg": 142,
        "reverse": false
      },
      "controlPoints": [],
      "color": "#CA9BCC",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "ml8b3cec-6nzv0r",
      "name": "Path 2",
      "endPoint": {
        "x": 30.5,
        "y": 60.5,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [
        {
          "x": 50.582857142857144,
          "y": 63.51428571428574
        }
      ],
      "color": "#58B8C5",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "ml8b7358-i5opd0",
      "name": "Path 3",
      "endPoint": {
        "x": 25,
        "y": 60.5,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [],
      "color": "#88699A",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "ml8b9amw-wlcf9d",
      "name": "Path 4",
      "endPoint": {
        "x": 54,
        "y": 80,
        "heading": "tangential",
        "reverse": true
      },
      "controlPoints": [],
      "color": "#BCA658",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "ml8bb11h-jf0ufl",
      "name": "Path 5",
      "endPoint": {
        "x": 21,
        "y": 65,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [],
      "color": "#766A9B",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "ml8bbmxj-rcnrlf",
      "name": "Path 6",
      "endPoint": {
        "x": 11,
        "y": 56,
        "heading": "linear",
        "reverse": false,
        "endDeg": 135,
        "startDeg": 135
      },
      "controlPoints": [
        {
          "x": 16.5,
          "y": 57.5
        }
      ],
      "color": "#7A56C5",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 144,
          "y": 70
        },
        {
          "x": 144,
          "y": 144
        },
        {
          "x": 120,
          "y": 144
        },
        {
          "x": 138,
          "y": 119
        },
        {
          "x": 138,
          "y": 70
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6,
          "y": 119
        },
        {
          "x": 25,
          "y": 144
        },
        {
          "x": 0,
          "y": 144
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 7,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    },
    {
      "id": "triangle-3",
      "name": "",
      "vertices": [
        {
          "x": 72.34285714285716,
          "y": 0.7542857142857233
        },
        {
          "x": 140.77714285714285,
          "y": 1.57714285714286
        },
        {
          "x": 142.7657142857143,
          "y": 143.52
        },
        {
          "x": 71.94285714285715,
          "y": 144.35428571428574
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-uqqf6khhuz"
    },
    {
      "kind": "path",
      "lineId": "ml8b3cec-6nzv0r"
    },
    {
      "kind": "path",
      "lineId": "ml8b7358-i5opd0"
    },
    {
      "kind": "path",
      "lineId": "ml8b9amw-wlcf9d"
    },
    {
      "kind": "path",
      "lineId": "ml8bb11h-jf0ufl"
    },
    {
      "kind": "path",
      "lineId": "ml8bbmxj-rcnrlf"
    }
  ],
  "settings": {
    "xVelocity": 75,
    "yVelocity": 65,
    "aVelocity": 3.141592653589793,
    "kFriction": 0.1,
    "rWidth": 18,
    "rHeight": 18,
    "safetyMargin": 1,
    "maxVelocity": 40,
    "maxAcceleration": 30,
    "maxDeceleration": 30,
    "fieldMap": "decode.webp",
    "robotImage": "/robot.png",
    "theme": "auto",
    "showGhostPaths": false,
    "showOnionLayers": true,
    "onionLayerSpacing": 3,
    "onionColor": "#dc2626",
    "onionNextPointOnly": false
  },
  "version": "1.2.1",
  "timestamp": "2026-02-04T17:42:21.738Z"
}