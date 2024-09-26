document.getElementById("get-question-btn").addEventListener("click", function() {
    const userId = document.getElementById("user_id").value;
    const keyId = document.getElementById("key_id").value;
    const scaleType = document.getElementById("scale_type").value;

    fetch("/get_question", {
        method: "POST",
        headers: {
            "Content-Type": "application/json",
        },
        body: JSON.stringify({ user_id: userId, key_id: keyId, scale_type: scaleType }),
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            document.getElementById("question-text").innerText = data.question;
            document.getElementById("result").innerText = "Question fetched successfully!";
        } else {
            document.getElementById("result").innerText = "Error fetching question: " + data.error;
        }
    })
    .catch(error => {
        document.getElementById("result").innerText = "Failed to fetch question: " + error.message;
    });
});
