// Connect to ROS
var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
});

// Subscribe to the /current_question topic
function subscribeToQuestion() {
    var questionTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/current_question',
        messageType: 'CurrentQuestion'
    });

    questionTopic.subscribe(function(message) {
        document.getElementById('question_text').innerHTML = message.question;
    });
}
