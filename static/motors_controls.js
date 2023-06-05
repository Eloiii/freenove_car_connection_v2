const request = new XMLHttpRequest();
const CAR_IP = '138.250.156.78'

const motors_controls = document.querySelectorAll('.direction')
for (let control of motors_controls) {
    control.addEventListener('mousedown', e => {
        const direction = e.target.classList[0]
        switch (direction) {
            case "forward":
                send(`/setMotors?ip=${CAR_IP}&value=4000_4000_4000_4000`)
                break
            case "left":
                send(`/setMotors?ip=${CAR_IP}&value=4000_4000_-4000_-4000`)
                break
            case "right":
                send(`/setMotors?ip=${CAR_IP}&value=-4000_-4000_4000_4000`)
                break
            case "backward":
                send(`/setMotors?ip=${CAR_IP}&value=-4000_-4000_-4000_-4000`)
                break
        }
        console.log(direction + ' mousedown')
    })
    control.addEventListener('mouseup', e => {
        send(`/setMotors?ip=${CAR_IP}&value=0_0_0_0`)
    })
}

const buzzer = document.querySelector('.buzzer')
buzzer.addEventListener('mousedown', () => {
    send(`/toggleBuzzer?ip=${CAR_IP}&value=0`)
})
buzzer.addEventListener('mouseup', () => {
    send(`/toggleBuzzer?ip=${CAR_IP}&value=1`)
})

const servos_controls = document.querySelectorAll(".servo")
const servos_values = [90, 90]

for(let control of servos_controls) {
    control.addEventListener('mousedown', e => {
        const direction = e.target.classList[0]
        switch(direction) {
            case "forward":
                send(`/setServo?ip=${CAR_IP}&value=1_${++servos_values[1]}`)
                break
            case "left":
                send(`/setServo?ip=${CAR_IP}&value=0_${--servos_values[0]}`)
                break
            case "right":
                send(`/setServo?ip=${CAR_IP}&value=1_${++servos_values[0]}`)
                break
            case "backward":
                send(`/setServo?ip=${CAR_IP}&value=1_${--servos_values[1]}`)
                break
        }
    })
    control.addEventListener('mouseup', () => {
        // send(`/setServo?ip=${CAR_IP}&value=0_90`)
        // send(`/setServo?ip=${CAR_IP}&value=1_90`)
    })
}

function send(url) {
    request.open("GET", url, true)
    request.send()
}