#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyResponse
from threading import Thread
from flask import Flask, render_template, request, redirect, jsonify
from quori_osu.srv import GetQuestion, KeyID, KeyIDRequest

app = Flask(__name__)

# Global variables
latest_question = "Waiting for message..."
scale_type = "Triad"
gui_node = None

class GuiNode:
    """ROS Node for the GUI application."""

    def __init__(self):
        """Initialize the GUI Node."""
        rospy.init_node('gui_node')

        # Services to start and stop GUI
        self.start_service = rospy.Service('start_gui', Empty, self.start_gui)
        self.stop_service = rospy.Service('stop_gui', Empty, self.stop_gui)

        # Initialize the service client for /get_question
        rospy.wait_for_service('/get_question')
        self.get_question_service = rospy.ServiceProxy('/get_question', GetQuestion)

        # Initialize the service client for /key_id
        rospy.wait_for_service('/key_id')
        self.key_id_service = rospy.ServiceProxy('/key_id', KeyID)

    def start_gui(self, req):
        """Start the web-based GUI."""
        rospy.loginfo("Starting web-based GUI")
        # Launch the Flask server in a separate thread
        Thread(target=self.launch_web_server).start()
        return EmptyResponse()

    def stop_gui(self, req):
        """Stop the web-based GUI."""
        rospy.loginfo("Stopping web-based GUI")
        return EmptyResponse()

    def launch_web_server(self):
        """Launch the Flask web server."""
        app.run(host='0.0.0.0', port=5000)

    def request_question(self, selected_button):
        """Request a question from the ROS service."""
        global latest_question
        try:
            response = self.get_question_service(selected_button)
            latest_question = response.question
            rospy.loginfo(f"Received question: {latest_question}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def send_key_id(self, user_id, key_id):
        """Send the user ID, key ID, and scale type to the /key_id service."""
        try:
            key_id_srv = KeyIDRequest(user_id=user_id, key_id=key_id, scale_type=scale_type)
            response = self.key_id_service(key_id_srv)

            if response.success:
                rospy.loginfo(f"Service call successful: User ID: {user_id}, Key ID: {key_id}, Scale Type: {scale_type}")
                return True
            else:
                rospy.logerr("Service call failed. Success flag is false.")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to /key_id failed: {e}")
            return False


# Flask routes for the web-based GUI
@app.route('/')
def index():
    """Render the main GUI page."""
    return render_template('index.html', question=latest_question, scale_type=scale_type)

@app.route('/submit', methods=['POST'])
def submit():
    """Handle the form submission."""
    user_id = request.form.get('user_id')
    key_id = request.form.get('key_id')
    scale_type_toggle = request.form.get('scale_type_toggle')

    if scale_type_toggle:
        global scale_type
        scale_type = "Likert" if scale_type == "Triad" else "Triad"

    if gui_node.send_key_id(user_id, int(key_id)):
        return redirect('/')
    else:
        return "Service call failed.", 500

@app.route('/next_question', methods=['POST'])
def next_question():
    """Request the next question."""
    selected_button = int(request.form.get('selected_button'))
    gui_node.request_question(selected_button)
    return redirect('/')


if __name__ == '__main__':
    # Start the ROS node and Flask app
    gui_node = GuiNode()
    app.run(host='0.0.0.0', port=5000)

# #!/usr/bin/env python3

# import rospy
# from std_srvs.srv import Empty, EmptyResponse
# from threading import Thread
# from flask import Flask, render_template, request, redirect
# from quori_osu.srv import GetQuestion, KeyID, KeyIDRequest

# app = Flask(__name__)

# # Global variables
# lastest_question = "Waiting for message..."
# scale_type = "Triad"
# gui_node = None

# class GuiNode:
#     """ROS Node for the GUI application."""

#     def __init__(self):
#         """Initialize the GUI Node."""
#         rospy.init_node('gui_node')
#         self.latest_question = "Waiting for message..."

#         # Services to start and stop GUI
#         self.start_service = rospy.Service('start_gui', Empty, self.start_gui)
#         self.stop_service = rospy.Service('stop_gui', Empty, self.stop_gui)

#         # Initialize the service client for /get_question
#         self.get_question_service = rospy.ServiceProxy('/get_question', GetQuestion)

#         # Initialize the service client for /key_id
#         rospy.wait_for_service('/key_id')
#         self.key_id_service = rospy.ServiceProxy('/key_id', KeyID)

#     def start_gui(self, req):
#         """Start the web-based GUI."""
#         rospy.loginfo("Starting web-based GUI")
#         # Launch the Flask server in a separate thread
#         Thread(target=self.launch_web_server).start()
#         return EmptyResponse()

#     def stop_gui(self, req):
#         """Stop the web-based GUI."""
#         rospy.loginfo("Stopping web-based GUI")
#         return EmptyResponse()

#     def launch_web_server(self):
#         """Launch the Flask web server."""
#         app.run(host='0.0.0.0', port=5000)

#     def request_question(self, selected_button):
#         """Request a question from the ROS service."""
#         global lastest_question
#         try:
#             response = self.get_question_service(selected_button)
#             lastest_question = response.question
#             rospy.loginfo(f"Received question: {lastest_question}")
#         except rospy.ServiceException as e:
#             rospy.logerr(f"Service call failed: {e}")

#     def send_key_id(self, user_id, key_id):
#         """Send the user ID, key ID, and scale type to the /key_id service."""
#         try:
#             # Create the request for the KeyID service
#             key_id_srv = KeyIDRequest()
#             key_id_srv.user_id = user_id
#             key_id_srv.key_id = key_id
#             key_id_srv.scale_type = scale_type

#             # Call the key_id_service with the request
#             response = self.key_id_service(key_id_srv)

#             if response.success:
#                 rospy.loginfo(f"Service call successful: User ID: {user_id}, Key ID: {key_id}, Scale Type: {scale_type}")
#                 return True
#             else:
#                 rospy.logerr("Service call failed. Success flag is false.")
#                 return False
#         except rospy.ServiceException as e:
#             rospy.logerr(f"Service call to /key_id failed: {e}")
#             return False


# # Flask routes for the web-based GUI
# @app.route('/')
# def index():
#     """Render the main GUI page."""
#     return render_template('index.html', question=lastest_question, scale_type=scale_type)

# @app.route('/submit', methods=['POST'])
# def submit():
#     """Handle the form submission."""
#     user_id = request.form.get('user_id')
#     key_id = request.form.get('key_id')
#     scale_type_toggle = request.form.get('scale_type_toggle')

#     if scale_type_toggle:
#         global scale_type
#         scale_type = "Likert" if scale_type == "Triad" else "Triad"

#     if gui_node.send_key_id(user_id, int(key_id)):
#         return redirect('/')
#     else:
#         return "Service call failed.", 500

# @app.route('/next_question', methods=['POST'])
# def next_question():
#     """Request the next question."""
#     selected_button = int(request.form.get('selected_button'))
#     gui_node.request_question(selected_button)
#     return redirect('/')


# if __name__ == '__main__':
#     # Start the ROS node and Flask app
#     gui_node = GuiNode()
#     app.run(host='0.0.0.0', port=5000)