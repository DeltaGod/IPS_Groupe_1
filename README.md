# IPS_Groupe_1

## UART Communication commands:
`:Dxxx` = change duty cycle  
`:S` = start data stream  
`:S0` = stop data stream  
`:A` = stop everything (`duty_cycle=0` & stop data stream)  
`:Xxxxx` = send temperature setpoint (ex: `:X0305`=30,5°C)  

## UART data formatting
`\n` = EOL -> to detect when 1 line of data ends  
line format: `Current;Temperature;DutyCycle;Power\n`  