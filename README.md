# IPS_Groupe_1

## How to use ?
1. Plug the card.  
2. You may need to change the port at the bottom of `main.py`, depending on which one the card is connected to.  
3. Launch `main.py`.  
4. If you have issues, you can send me a message.  

## UART Communication commands:
`:Dxxx` = change duty cycle  
`:S` = start data stream  
`:S0` = stop data stream  
`:A` = stop everything (`duty_cycle=0` & stop data stream)  
`:Xxxxx` = send temperature setpoint (ex: `:X0305`=30,5°C)  
`:C1` = confirm temperature setpoint to follow

## UART data formatting
`\n` = EOL -> to detect when 1 line of data ends  
line format: `Current;Temperature;DutyCycle;Power\n`  