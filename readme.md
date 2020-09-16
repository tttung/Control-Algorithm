# BalanceCar

# flightControl

# Mppt

# navigation

# PID
$u(t)=K_{p} e(t)+K_{i} \int_{0}^{t} e(\tau) d \tau+K_{d} \frac{d e(t)}{d t}$
* P：比例控制，就是把实际偏差放大或者缩小一定倍数，然后作为输出
* I：积分控制，实际上就是看多个P的叠加值，只要有偏差，它就不停的叠加（有正负），不让偏差变成0不罢休
* D:根据偏差变化量的大小去输出，相当于“预测”了偏差的变化方向去做提前调整

PID输出主要取决于需要的类型，可以做一个量程转换或者用自带的量程转换，转换成需要的类型如4-20mA或者±10V等。本身输出范围不是0.0.-1.0，而是0-100%中间可以很多位。

## 位置式 PID
$u(k)=K_{p} e_{k}+K_{i} \sum_{i=1}^{k} e(i) \Delta t+K_{d} \frac{e(k)-e(k-1)}{\Delta t}$
* 位置式PID计算结果表示控制信号的绝对大小

## 增量式PID
$\Delta u(k)=K_{p}(e(k)-e(k-1))+K_{i} e(k)+K_{d}(e(k)-2 e(k-1)+e(k-2))$
* 增量式PID计算结果表示控制信号的相对大小

# filter
## KalmanFilter
## FIRFilter
## ellipsefilter
