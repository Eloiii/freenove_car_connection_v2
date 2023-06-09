const request = new XMLHttpRequest();
const CAR_IP = '138.250.156.219'

const motor_controls = document.querySelectorAll('.direction')
for (let control of motor_controls) {
    control.addEventListener('mousedown', e => {
        const direction = e.target.classList[0]
        switch (direction) {
            case "forward":
                send(`/setMotors?ip=${CAR_IP}&value=2000_2000_2000_2000`)
                break
            case "left":
                send(`/setMotors?ip=${CAR_IP}&value=-2000_-2000_2000_2000`)
                break
            case "right":
                send(`/setMotors?ip=${CAR_IP}&value=2000_2000_-2000_-2000`)
                break
            case "backward":
                send(`/setMotors?ip=${CAR_IP}&value=-2000_-2000_-2000_-2000`)
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
let servos_values = [90, 90]

let servoTimer

for (let control of servos_controls) {
    control.addEventListener('mousedown', e => {
        const direction = e.target.classList[0]
        if (direction === "home") {
            send(`/setServo?ip=${CAR_IP}&value=0_90`)
            setTimeout(() => send(`/setServo?ip=${CAR_IP}&value=1_90`), 50)
        }
        servoTimer = setInterval(() => {
            switch (direction) {
                case "forward":
                    send(`/setServo?ip=${CAR_IP}&value=1_${servos_values[1] += 2}`)
                    break
                case "left":
                    send(`/setServo?ip=${CAR_IP}&value=0_${servos_values[0] -= 2}`)
                    break
                case "right":
                    send(`/setServo?ip=${CAR_IP}&value=0_${servos_values[0] += 2}`)
                    break
                case "backward":
                    send(`/setServo?ip=${CAR_IP}&value=1_${servos_values[1] -= 2}`)
                    break

            }
        }, 100)

    })
    control.addEventListener('mouseup', () => {
        if (servoTimer) {
            clearInterval(servoTimer)
        }
    })
}

document.addEventListener('keydown', e => {
    if (e.code === 'Enter') {
        const target = e.target
        const rawTxt = target.value
        const rbgValues = rawTxt.replaceAll(' ', '').replaceAll(',', '_') || '0_0_0'
        const led = target.id.split('_')[0]
        send(`/setLED?ip=${CAR_IP}&value=${led}_${rbgValues}`)
    }
})


function send(url) {
    request.open("GET", url, true)
    request.send()
}