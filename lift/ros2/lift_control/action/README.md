int32 command
0: Stop
1: Go Up
2: Go Down
---
int32 completion
0: Moving
1: Reached
---
int32 status
0: Moving
1: Reached
---
int32 Lift Status
001: Stationary at First Level (Only Go Up can be accepted)
100: Stationary at Second Level (Only Go Down can be accepted)
011: Moving to First Level (Only Stop can be accepted)
110: Moving to Second Level (Only Stop can be accepted)
010: Stationary Between Levels (Go Up and Go Down accepted)