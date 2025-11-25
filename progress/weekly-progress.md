# About this document

This is a team document where you record your weekly progress. It is written in Markdown, which allows you to format content in a simple and readable way. The document is rendered directly in GitHub without the need for a compiler (unlike LaTeX). The syntax is relatively easy. An overview of commonly used Markdown syntax can be found here:  
https://www.markdownguide.org/basic-syntax/

Below you find an example of the sections that must be included in each weekly progress report.

Use the **same document for all weeks**. For each week, use the **same headers and subheaders**.

Use the GitHub repository to store important project files (code, visuals including videos and figures, data, etc.). If necessary refer to those files in this document using a hyperlink. 

---

# Week 3

## 1. Progress description
Describe the progress of your work for this week. Keep it clear and concise.

## 2. Code

You can display code using three backticks (```) followed by an optional language extension.  
For example:  
- For C/C++ code, use `c` or `cpp`.  
- For Python, use `python`.


#### Example:

```c
// Header file for input/output functions
#include <stdio.h>

// Main function: entry point for execution
int main() {

    // Print a message to the console
    printf("Hello World");

    return 0;
}
```
## 3. Measurment protocol
There will be measured at 8 different PWM values in both directions.
The values will be: 50, 80, 110, 140, 170, 200, 230, 255

One will peform two measurements per PWM value, one while the wheel is spinning to the right and one while the wheel is spinning to the left. In total there will be 16 measurments. The total measurement lasts 90 seconds, so each individual measurment lasts 5.5 seconds. 

Because of the fact that the motor can rotate in two directions one will first measure all the different PWM values while the wheel turns to the right. Then, automaticly, the wheel stops spinning and switch in turning to the left. All the PWM values will be measured again and this is how to account for this problem.

The following graph is to be expected:
 <img width="752" height="452" alt="image" src="https://github.com/user-attachments/assets/f1e5559b-7ba2-4518-9c76-23f09a0c95c6" />
This is expected because at the maximum PWM value, one would anticipate to reach the maximum RPM count. 

To upload and modify the code arduino IDE is used. A serial moniter will print the data and a python file will read the serial moniter and generate a csv file from the data and, if possible, creates plots.

- arduino IDE voor uploaden en bewerken van code
- serial moniter print data uit\
- python bestand leest serial moniter en maakt csv file van uitgelezen data en eventueel plots

## 4. Results
Present your results here. This may include tables, figures, or charts.
Add charts and other visuals to the `visuals` folder in the GitHub repository and reference them in this document if needed.


## 5. Reflection 
What did you learn this week?
What should you focus on or improve in the coming weeks of the project?
