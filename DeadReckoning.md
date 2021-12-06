## Dead Reckoning
![](attachments/Dead%20reckoning.png)
已知$^B_WT$、$^{B^{\prime}}_BT$，求$^{B^{\prime}}_WT$。
$$
\begin{aligned}
^{B^{\prime}}_WT&=^B_WT\cdot^{B^{\prime}}_BT \\
&=\begin{bmatrix}
cos\alpha & -sin\alpha & 0 & x \\
sin\alpha & cos\alpha & 0 & y \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}\cdot\begin{bmatrix}
cos\Delta\alpha & -sin\Delta\alpha & 0 & {\Delta}x \\
sin\Delta\alpha & cos\Delta\alpha & 0 & {\Delta}y \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix} \\
&=\begin{bmatrix}
cos{\alpha}cos\Delta\alpha-sin{\alpha}sin\Delta\alpha & -cos{\alpha}sin\Delta\alpha-sin{\alpha}cos\Delta\alpha & 0 & {\Delta}xcos{\alpha}-{\Delta}ysin\alpha+x \\
sin{\alpha}cos\Delta\alpha+cos{\alpha}sin\Delta\alpha & -sin{\alpha}sin\Delta\alpha+cos{\alpha}cos\Delta\alpha & 0 & {\Delta}xsin{\alpha}+{\Delta}ycos\alpha+y \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix} \\
&=\begin{bmatrix}
cos(\alpha+\Delta\alpha) & -sin(\alpha+\Delta\alpha) & 0 & {\Delta}xcos{\alpha}-{\Delta}ysin\alpha+x \\
sin(\alpha+Delta\alpha) & cos(\alpha+\Delta\alpha) & 0 & {\Delta}xsin{\alpha}+{\Delta}ycos\alpha+y \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
\end{aligned}
$$