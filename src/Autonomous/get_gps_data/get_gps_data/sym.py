from flask import Flask, render_template, jsonify, request
import folium
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt6.QtCore import QUrl
from PyQt6.QtWebEngineWidgets import QWebEngineView
import threading
import sys
import os


# Ensure Flask always finds the templates folder
template_dir = os.path.join(os.path.dirname(os.getcwd()), "WRoverSoftware/src/get_gps_data/get_gps_data/templates")
print(template_dir)
app = Flask(__name__, template_folder=template_dir)

# Robot and goal locations   Next to Mechanical Engineering : 43.0721443104249, -89.41169303680519
robot_location = {"lat": 43.07235320367639, "lon": -89.41159304772383} #INITIALIZES ROBOT LOCATION

# takes a 2d array in the form lat lon
def set_goals(x):
    global goal_locations
    goal_locations = []
    for i in x:
        goal_locations.append({"lat": str(i[0]), "lon": str(i[1]), "name": "Goal " + str(x.index(i) + 1)})

goal_locations = [
    {"lat": 43.07235320367639, "lon": -89.41159304772383, "name": "Front of MEC E"},
    {"lat": 38.374, "lon": -110.828, "name": "UTAH"},
]

@app.route("/map")
def home():
    return render_template("actual_map.html")

@app.route("/get_location")
def get_location():
    """Returns the current robot location as JSON."""
    return jsonify(robot_location)

@app.route("/update_location", methods=["POST"])
def update_location():
    """Updates the robot's location via a POST request."""
    global robot_location
    data = request.json
    #print(data)
    robot_location["lat"] = data.get("lat")
    robot_location["lon"] = data.get("lon")
    print(robot_location)
    return jsonify({"message": "Location updated!"})

@app.route("/get_goals")
def get_goals():
    """Returns goal locations."""
    return jsonify(goal_locations)


# Function to generate the map.html file
def generate_map():
    m = folium.Map(location=[robot_location["lat"], robot_location["lon"]], zoom_start=100)

    # Add robot marker (this will be updated dynamically)
    folium.Marker(
        location=[robot_location["lat"], robot_location["lon"]],
        popup="Robot",
        icon=folium.Icon(color="blue")
    ).add_to(m)

    # Add goal markers
    for goal in goal_locations:
        folium.Marker(
            location=[goal["lat"], goal["lon"]],
            popup=goal["name"],
            icon=folium.Icon(color="red")
        ).add_to(m)

    m.save(os.path.join(os.getcwd(), "templates", "map.html"))

# Run Flask in a separate thread so it doesn't block Tkinter
def run_flask():
    #IMPORTANT  generate_map() - must manually generate the map on the fist run then change map.html - DO THIS
    app.run(debug=True, port=5000, use_reloader=False)

class RobotTracker(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Tracker")
        self.setGeometry(100, 100, 1024, 768)  # Window size

        # Create a web view
        self.browser = QWebEngineView()
        self.browser.setUrl(QUrl("http://127.0.0.1:5000/map"))  # Load your web app

        # Set central widget
        central_widget = QWidget()
        layout = QVBoxLayout()
        layout.addWidget(self.browser)
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)

def run_gui():
    app = QApplication(sys.argv)
    window = RobotTracker()
    window.show()
    sys.exit(app.exec())

def main(args=None):
    print("Current Working Directory:", os.getcwd())
    threading.Thread(target=run_flask, daemon=True).start()
    run_gui()

if __name__ == "__main__":
    main()