set datafile separator ","
plot "pins.log" using "time":"clk" with steps, "" using "time":(column("data")+1.1) with steps,"" using "time":(column("datain")+2.2) with steps
pause -1