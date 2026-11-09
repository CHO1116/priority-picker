# 검출된 큐브 1개 (좌표는 로봇 base 기준, 단위 mm)
string color          # red / blue / green / yellow / unknown
float64 x_mm
float64 y_mm
float64 z_mm          # 큐브 윗면 높이
float64 yaw_deg       # -45 ~ 45 (정육면체는 90도 대칭)
float32 confidence    # 0.0 ~ 1.0
float32 min_gap_mm    # 가장 가까운 다른 큐브와의 간격 (그리퍼 손가락이 들어갈 틈)
