let CLIENT_INDEX

connectionDiv = document.getElementById("connection")
carIpInput = document.getElementById("car_ip")

carIpInput.addEventListener('keydown', async e => {
    if (e.code === 'Enter') {
        const data = await send(`/connect/${carIpInput.value}`)
        CLIENT_INDEX = data['client_index']
        connectionDiv.style.color = 'green'
    }
})
carIndexInput = document.getElementById("car_index")
carIndexInput.addEventListener('keydown', async e => {
    if (e.code === 'Enter') {
        CLIENT_INDEX = carIndexInput.value
        connectionDiv.style.color = 'green'
    }
})


const motor_controls = document.querySelectorAll('.direction')
for (let control of motor_controls) {
    control.addEventListener('mousedown', async e => {
        const direction = e.target.classList[0]
        switch (direction) {
            case "forward":
                await send(`/setMotors?ci=${CLIENT_INDEX}&value=4000_4000_4000_4000`)
                break
            case "left":
                await send(`/setMotors?ci=${CLIENT_INDEX}&value=4000_4000_-4000_-4000`)
                break
            case "right":
                await send(`/setMotors?ci=${CLIENT_INDEX}&value=-4000_-4000_4000_4000`)
                break
            case "backward":
                await send(`/setMotors?ci=${CLIENT_INDEX}&value=-4000_-4000_-4000_-4000`)
                break
        }
    })
    control.addEventListener('mouseup', async e => {
        await send(`/setMotors?ci=${CLIENT_INDEX}&value=0_0_0_0`)
    })
}

const buzzer = document.querySelector('.buzzer')
buzzer.addEventListener('mousedown', async () => {
    await send(`/toggleBuzzer?ci=${CLIENT_INDEX}&value=0`)
})
buzzer.addEventListener('mouseup', async () => {
    await send(`/toggleBuzzer?ci=${CLIENT_INDEX}&value=1`)
})

const servos_controls = document.querySelectorAll(".servo")
let servos_values = [90, 90]

let servoTimer

for (let control of servos_controls) {
    control.addEventListener('mousedown', async e => {
        const direction = e.target.classList[0]
        if (direction === "home") {
            await send(`/setServo?ci=${CLIENT_INDEX}&value=0_90`)
            setTimeout(async () => await send(`/setServo?ci=${CLIENT_INDEX}&value=1_90`), 50)
        }
        servoTimer = setInterval(async () => {
            switch (direction) {
                case "forward":
                    await send(`/setServo?ci=${CLIENT_INDEX}&value=1_${servos_values[1] -= 2}`)
                    break
                case "left":
                    await send(`/setServo?ci=${CLIENT_INDEX}&value=0_${servos_values[0] -= 2}`)
                    break
                case "right":
                    await send(`/setServo?ci=${CLIENT_INDEX}&value=0_${servos_values[0] += 2}`)
                    break
                case "backward":
                    await send(`/setServo?ci=${CLIENT_INDEX}&value=1_${servos_values[1] += 2}`)
                    break

            }
        }, 100)

    })
    control.addEventListener('mouseup', () => {
        if (servoTimer) {
            clearInterval(servoTimer)
            servos_values = [90, 90]
        }
    })
}

const ledInputs = document.querySelectorAll(".led")

for (let ledInput of ledInputs) {
    ledInput.addEventListener('keydown', async e => {
        if (e.code === 'Enter') {
            const target = e.target
            const rawTxt = target.value
            const rbgValues = rawTxt.replaceAll(' ', '').replaceAll(',', '_') || '0_0_0'
            const led = target.id.split('_')[0]
            await send(`/setLED?ci=${CLIENT_INDEX}&value=${led}_${rbgValues}`)
        }
    })
}


async function send(url) {
    const response = await fetch(url)
    const data = response.json()
    return data
}
