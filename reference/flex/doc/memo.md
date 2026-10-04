Quaternion rotation
$$
\begin{align*}
    p' &= q^{-1}pq \\
       &=  (q_w, \quad -\bm{q_{xyz}}) \times (0, \quad \bm{p_{xyz}}) \times(q_w, \quad \bm{q_{xyz}}) \\
       &= (q_w, \quad -\bm{q_{xyz}}) \times (-p_{xyz} \cdot q_{xyz}, \quad p_{xyz}q_w + (p_{xyz} \times q_{xyz})) \\
\end{align*}
$$

$$
    \begin{align*}
    p'_{xyz} = & q_w^2 p_{xyz} + q_w(p_{xyz} \times q_{xyz}) + (p_{xyz} \cdot q_{xyz})q_{xyz} \\
               & - q_w(q_{xyz} \times p_{xyz}) - (q_{xyz} \times (p_{xyz} \times q_{xyz})) \\
             = & q_w^2 p_{xyz} + 2 * q_w (p_{xyz} \times q_{xyz}) + (p_{xyz} \cdot q_{xyz}) q_{xyz} \\
             & - (q_{xyz} \cdot q_{xyz})p_{xyz} + (q_{xyz} \cdot p_{xyz})q_{xyz} \\
             = & (2q_w^2 - 1) p_{xyz} + 2 q_w(p_{xyz} \times q_{xyz}) + 2(p_{xyz} \cdot q_{xyz})q_{xyz}
    \end{align*}
$$