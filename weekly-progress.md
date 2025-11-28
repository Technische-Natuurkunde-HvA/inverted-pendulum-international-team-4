# About this document

This is a team document where you record your weekly progress. It is written in Markdown, which allows you to format content in a simple and readable way. The document is rendered directly in GitHub without the need for a compiler (unlike LaTeX). The syntax is relatively easy. An overview of commonly used Markdown syntax can be found here:  
https://www.markdownguide.org/basic-syntax/

Below you find an example of the sections that must be included in each weekly progress report.

Use the **same document for all weeks**. For each week, use the **same headers and subheaders**.

Use the GitHub repository to store important project files (code, visuals including videos and figures, data, etc.). If necessary refer to those files in this document using a hyperlink. 

---

# Week 3

## 1. Progress description
In the Netherlands we worked on the pendulum this week. We started with a spining wheel with no motion in the arm. After we had some difficulties with the code we managed to get the pendulum to move. We started with a swinging arm (see the video in results) but Somtimes we were able to balance it but it was not consistent. 

## 2. Code


## 3. Measurment protocol
There will be measured at 8 different PWM values in both directions.
The values will be: 50, 80, 110, 140, 170, 200, 230, 255

One will peform two measurements per PWM value, one while the wheel is spinning to the right and one while the wheel is spinning to the left. In total there will be 16 measurments. The total measurement lasts 90 seconds, so each individual measurment lasts 5.5 seconds. 

Because of the fact that the motor can rotate in two directions one will first measure all the different PWM values while the wheel turns to the right. Then, automaticly, the wheel stops spinning and switch in turning to the left. All the PWM values will be measured again and this is how to account for this problem.

The following graph is to be expected:
 <img width="752" height="452" alt="image" src="https://github.com/user-attachments/assets/f1e5559b-7ba2-4518-9c76-23f09a0c95c6" />
This is expected because at the maximum PWM value, one would anticipate to reach the maximum RPM count. 

To upload and modify the code arduino IDE is used. A serial moniter will print the data and a python file will read the serial moniter and generate a csv file from the data and, if possible, creates plots.


## 4. Results
The following graph shows the results:
![WhatsApp Image 2025-11-28 at 14 32 18](https://github.com/user-attachments/assets/b4ce2a50-342c-4867-a038-0a8fc6dbc08e)


This graph deviates from the expected graph. This is because in reality there is friction

Here is a video of the Flywheel:


https://github.com/user-attachments/assets/1bf1f06c-9694-4833-a927-8c63ff81f303


## 5. Reflection 
Our improvmends for the next week will be to make a new design for the fly wheel. We want to make the wheel bigger and thinner. If we do this we would have a bigger moment of inertia. We also want to try to balance the wheel consistently. 


