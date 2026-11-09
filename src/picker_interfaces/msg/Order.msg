uint8 URGENT=0
uint8 NORMAL=1

int32 order_id
uint8 priority        # URGENT / NORMAL (숫자가 작을수록 먼저)
string box_id         # 출고 박스 (poses.yaml의 boxes 키)
OrderItem[] items
string status         # PENDING / IN_PROGRESS / DONE / PARTIAL
