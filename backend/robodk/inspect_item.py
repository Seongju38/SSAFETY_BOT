# 열려있는 rdk의 아이템 이름/타입 확인용
from robodk import robolink

RDK = robolink.Robolink()

# rdk 파일 자동으로 열기
# RDK.AddFile(r"..\simulation\AI_industry_site.rdk")

items = RDK.ItemList()
print("=== RoboDK Items ===")
for it in items:
    print(it.Name(), it.Type())
