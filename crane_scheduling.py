import matplotlib.pyplot as plt
from itertools import permutations
import copy

class Crane:
    def __init__(self, order):
        self.order = order
        self.cluster_set = {}  # 클러스터 관련 정보를 저장할 딕셔너리
        self.trans_dict = {}   # 트랜지션 관련 정보를 저장할 딕셔너리
        self.init_dict = {}    # 초기화 관련 정보를 저장할 딕셔너리
    
    def trace_x(self, a, b): # x추적 함수
        if a > b:
            temp= list(range(a, b - 1, -1)) #하강(x감소)하는경우
        elif a == b:
            temp= [a]
        else:
            temp= list(range(a, b + 1)) #상승(x증가)하는경우

        if len(temp) != 1: # 움직이는 경우 앞 뒤로 정지 좌표 추가
            temp.insert(0,temp[0])
            temp.append(temp[-1])
        elif len(temp) == 1: # 제자리에 있는 경우 정지좌표 한 번만 추가
            temp.append(temp[0])

        return temp
    
    def time_check(self,list1,list2): #충돌시 밀어서 시간구하는 함수- 충돌하지 않는 경우도 add=0 으로 result 나옴
        add=0
        compare=0
        is_collision = True

        while True:
            # 상승하강, 하강상승 경우 동시에 고려
            if all(list1[j] <= list1[j+1] for j in range(len(list1)-1)) and all(list2[j] >= list2[j+1] for j in range(len(list2)-1)) or all(list1[j] >= list1[j+1] for j in range(len(list1)-1)) and all(list2[j] <= list2[j+1] for j in range(len(list2)-1)):
                for q in range(len(list1)): #compare를 계속 update해주기위함, list1의 길이만큼만 고려하면됨
                    if len(list1)>=len(list2): #list1이 더 길때
                        compare=min(len(list1)-add,len(list2))
                    else: #list2가 더 길때
                        compare=len(list1)-add
                    if compare == 1:
                        for k in range (1,compare+1):
                            if k>=1:
                             if list1[k+add-1]==list2[k-1] or list1[k+add]==list2[k] :
                                add += 1
                                break

                    else:
                        for k in range (compare):
                          if k>=1:
                            if list1[k+add]<=list2[k-1] and list2[k]<=list1[k+add-1]:
                                add += 1
                                break
                            elif list1[k+add-1]==list2[k-1] or list1[k+add]==list2[k] :
                                add += 1
                                break
                    is_collision=False

            if is_collision == False :
                result=max(add+len(list2)-1,max(len(list1),len(list2))-1)
                return result, add
                break


        #둘다 상승, 하강이거나 둘중 하나가 제자리인 경우
            for q in range(len(list1)): #compare를 계속 update해주기위함, list1의 길이만큼만 고려하면됨
                if len(list1)>=len(list2): #list1이 더 길때
                  compare=min(len(list1)-add,len(list2))
                else: #list2가 더 길때
                  compare=len(list1)-add
                if compare == 1:
                    for k in range (1,compare+1):
                      if k>=1:
                        if list1[k+add-1]==list2[k-1] or list1[k+add]==list2[k] :
                          add += 1
                          break
                else:
                    for k in range (compare):
                      if k>=1:
                        if list1[k+add-1]==list2[k-1] or list1[k+add]==list2[k] :
                          add += 1
                          break
                is_collision=False

            if is_collision == False :
                result=max(add+len(list2)-1,max(len(list1),len(list2))-1)
                return result, add
                break

    def cluster(self, order1, order2):
        result = []
        cluster=[]
        x1=[]
        x2=[]
        t1=[]
        t2=[]

        #order1 -> trajectory로 변환해서 result 에 집어넣기
        a = order1[0] #시작점
        b = order1[1] #도착지점
        index_order1=order1[-1] #order 구분위한 index 저장
        order1=self.trace_x(a,b)
        a = order2[0] #시작점
        b = order2[1] #도착지점
        index_order2=order2[-1]#order 구분위한 index 저장
        order2=self.trace_x(a,b)


        t_cluster,add =self.time_check(order1,order2) # 전체소요시간과 add 구하기

        for i in range(len(order1)):
            x1.append(order1[i])
        for i in range(len(order2)):
            x2.append(order2[i])

        end1= order1[len(order1)-1]
        end2= order2[len(order2)-1]
        start2 = order2[0]
        i=0

        # x1 남은 좌표 찍어주기
        # end1 과 start2 가 같은 경우 예외처리
        if end1==start2 and add == len(order1):
            if order1[add-3] >= order1[add-2]: #order1 감소, 정지
                for i in range(len(order2)):
                  x1.append(max(end1+1, order2[i]+1)) # list2 상승인 경우 따라가고 하강인 경우 end 값 유지
            elif order1[add-3] < order1[add-2]: #list1 상승
                for i in range(len(order2)):
                  x1.append(min(end1-1,order2[i]-1 ))

        # order1 과 order2 가 겹치는 모든 경우
        elif add+len(order2)>len(order1): # 2 가 1보다 늦게 끝나는 경우 -> 1의 남은 좌표 찍어주기
            if end1> order2[len(order2)-(t_cluster-len(order1))-2]: # 끝나는 지점에서 1이 2보다 위에 있는 경우
                while len(x1)-1 != t_cluster:
                  x1.append(max(end1, order2[len(order2)-(t_cluster-len(order1))-1+i]+1))
                  i=i+1
            elif end1< order2[len(order2)-(t_cluster-len(order1))-2]: # 끝나는 지점에서 2가 1보다 위에 있는 경우
                while len(x1)-1 != t_cluster:
                  x1.append(min(end1, order2[len(order2)-(t_cluster-len(order1))-1+i]-1))
                  i=i+1

        elif add+len(order2)<len(order1): # 1이 2보다 늦게 끝나는 경우-> 2 의 남은 좌표 찍어주기
            if end2> order1[add+len(order2)-1]: # 끝나는 지점에서 2가 1보다 위에 있는 경우
                while (add+len(x2)-1) != t_cluster:
                  x2.append(max(end2, order1[add+len(order2)+i]+1))
                  i=i+1
            elif end2< order1[add+len(order2)-1]: #끝나는 지점에서 1이 2보다 위에 있는 경우
                while (add+len(x2)-1) != t_cluster:
                  x2.append(min(end2, order1[add+len(order2)+i]-1))
                  i=i+1

        for i in range(t_cluster+1):
            t1.append(i)
        for i in range(add, t_cluster+1):
            t2.append(i)

        return ([order1,order2,add,t_cluster,[x1[len(x1)-1],x2[len(x2)-1]]])

    def get_trans(self, i_,j_,i,j,cluster_set):
        s1=cluster_set[(i,j)][0][0] # 뒷 클러스터 order1 시작점
        s2=cluster_set[(i,j)][1][0] # 뒷 클러스터 order2 시작점
        delay=cluster_set[(i,j)][2] # 뒷 클러스터 add
        af1=cluster_set[(i_,j_)][4][0] # 앞 클러스터 끝점(order1)
        af2=cluster_set[(i_,j_)][4][1] # 앞 클러스터 끝점(order2)
        df1=cluster_set[(i,j)][4][0] # 뒤 클러스터 끝점(order1)
        df2=cluster_set[(i,j)][4][1] # 뒤 클러스터 끝점(order2)
        if df1>df2:
            time_t= max(abs(max(af1,af2)-s1),abs(min(af1,af2)-s2)-delay)
                    #order1종료 #order2종료#cluster2의 order1의 첫좌표                                        #cluster2의 order2의 첫좌표
        else: #cluster[j](뒤)에서 order1 이 order2 보다 아래에 존재
            time_t= max(abs(max(af1,af2)-s2)-delay,abs(min(af1,af2)-s1))

        return time_t

    def get_init(self, i,j,cluster_set):
        s1=cluster_set[(i,j)][0][0] # 클러스터 order1 시작점
        e1=cluster_set[(i,j)][0][-1] # 클러스터 order1 끝점
        s2=cluster_set[(i,j)][1][0] # 클러스터 order2 시작점
        e2=cluster_set[(i,j)][1][-1] # 클러스터 order2 끝점
        if abs(10 - s1) == abs(20 - s2) and abs(10 - s2) == abs(20 - s1):
          if abs(20-s1) > abs(20-s2) and e2>s2: 
            time_i = abs(20-s2)
          else:  
            time_i = max(abs(10 - s1),abs(10 - s2))
        else:  
            time_i= min(max(abs(10 - s1),abs(20 - s2)),max(abs(10 - s2),abs(20 - s1)))

        return time_i 

    def repeat(self, i, j, left, temp_dict,init_dict,trans_dict,cluster_set): 
        if len(left) == 2:
            min_time = init_dict[(i, j)] + cluster_set[(i, j)][3]
            temp_dict[(i, j, str(left))] = (min_time, [(i, j)])  # 기록 초기화
            return min_time, [(i, j)], temp_dict

        if (i, j, str(left)) in temp_dict: #이미 계산됐는지 검사
            return temp_dict[(i, j, str(left))]

        left_ = left.copy()
        left_.remove(i)
        left_.remove(j)
        min_time = float('inf')
        min_i_j = None
        min_order = None

        for i_ in left_:
            for j_ in left_:
                if i_ != j_:
                    if (i_, j_, str(left_)) in temp_dict:
                        min_time_ = temp_dict[(i_, j_, str(left_))][0] + trans_dict[(i_, j_), (i, j)] + cluster_set[(i, j)][3]
                    else:
                        min_time_, _, _ = self.repeat(i_, j_, left_, temp_dict,init_dict,trans_dict,cluster_set)
                        min_time_ += trans_dict[(i_, j_), (i, j)] + cluster_set[(i, j)][3]

                    if min_time_ < min_time:
                        min_time = min_time_
                        min_i_j = (i_, j_)
                        min_order = temp_dict[(i_, j_, str(left_))][1]

        order = [(i, j)] + min_order
        temp_dict[(i, j, str(left))] = (min_time, order)
        return min_time, order, temp_dict 

    def full_traj(self, list1, list2, add, result):
        t1=[]
        t2=[]
        x1=[]
        x2=[]

        for i in range(len(list1)):
            x1.append(list1[i])
        for i in range(len(list2)):
            x2.append(list2[i])

        end1= list1[len(list1)-1]
        end2= list2[len(list2)-1]
        start2 = list2[0]
        i=0

        # x1 남은 좌표 찍어주기
        # end1 과 start2 가 같은 경우 예외처리
        if end1==start2 and add == len(list1):
            if list1[add-3] >= list1[add-2]: #list1감소, 정지
                for i in range(len(list2)):
                    x1.append(max(end1+1, list2[i]+1))
            elif list1[add-3] < list1[add-2]: #list1 상승
                for i in range(len(list2)):
                    x1.append(min(end1-1,list2[i]-1 ))

        elif add+len(list2)>len(list1): # 2 가 1보다 늦게 끝나는 경우
            if end1> list2[len(list2)-(result-len(list1))-2]: # 끝나는 지점에서 1이 2보다 위에 있는 경우
                while len(x1)-1 != result:
                    x1.append(max(end1, list2[len(list2)-(result-len(list1))-1+i]+1))
                    i=i+1
            elif end1< list2[len(list2)-(result-len(list1))-2]: # 끝나는 지점에서 2가 1보다 위에 있는 경우
                while len(x1)-1 != result:
                    x1.append(min(end1, list2[len(list2)-(result-len(list1))-1+i]-1))
                    i=i+1

        elif add+len(list2)<len(list1): # 1이 2보다 늦게 끝나는 경우
            if end2> list1[add+len(list2)-1]: # 끝나는 지점에서 2가 1보다 위에 있는 경우
                while (add+len(x2)-1) != result:
                    x2.append(max(end2, list1[add+len(list2)+i]+1))
                    i=i+1
            elif end2< list1[add+len(list2)-1]: #끝나는 지점에서 1이 2보다 위에 있는 경우
                while (add+len(x2)-1) != result:
                    x2.append(min(end2, list1[add+len(list2)+i]-1))
                    i=i+1

        # crane2 의 전 경로 찍는 경우
        if add>0: # add가 1이상일때만 crane2의 전경로가 필요함
        # 예외처리 end1 과 start2 가 동일한 경우
            if end1==start2 and add == len(list1):
                if list1[add-3] >= list1[add-2]: #list1감소, 정지
                    for i in range(add):
                        x2.insert(0,end1-1)
                elif list1[add-3] < list1[add-2]: # list1 상승
                    for i in range(add):
                        x2.insert(0,end1+1)

            elif add == len(list1): #예외처리(list1의 길이만큼밀때)- 2가 시작할 때 위치관계를 알 수 없음
                if list1[add-3] > list1[add-2]: #list1감소
                    for i in range(add):
                        x2.insert(i,max(start2, list1[i]+1))
                elif list1[add-3] < list1[add-2]: #list1증가
                    for i in range(add):
                        x2.insert(i,min(start2, list1[i]-1))
            else:
                if start2 > list1[add]: #2가 1위에 존재하는 경우  
                    for i in range(add):
                        x2.insert(i,max(start2, list1[i]+1))
                elif start2 < list1[add]: #1이 2 위에 존재하는 경우 
                    #1이 2 위에 존재하는 경우 
                    for i in range(add):
                        x2.insert(i,min(start2, list1[i]-1))

        for i in range(result+1): # 시간을 result 길이로
            t1.append(i)
            t2.append(i)
        return  (x1, x2) 
    
    def full_traj2(self, list3 ,list4 ,list1, list2, add, result ,tran):
              #앞클  #앞클  #뒷클  #뒷클
        t1=[]
        t2=[]
        x1=[]
        x2=[]

        for i in range(len(list1)):
            x1.append(list1[i])
        for i in range(len(list2)):
            x2.append(list2[i])

        end1= list1[len(list1)-1]
        end2= list2[len(list2)-1]
        start2 = list2[0]
        i=0

        # x1 남은 좌표 찍어주기
        # end1 과 start2 가 같은 경우 예외처리
        if end1==start2 and add == len(list1):
            if list1[add-3] >= list1[add-2]: #list1감소, 정지
                for i in range(len(list2)):
                    x1.append(max(end1+1, list2[i]+1))
            elif list1[add-3] < list1[add-2]: #list1 상승
                for i in range(len(list2)):
                    x1.append(min(end1-1,list2[i]-1 ))

        elif add+len(list2)>len(list1): # 2 가 1보다 늦게 끝나는 경우
            if end1> list2[len(list2)-(result-len(list1))-2]: # 끝나는 지점에서 1이 2보다 위에 있는 경우
                while len(x1)-1 != result:
                    x1.append(max(end1, list2[len(list2)-(result-len(list1))-1+i]+1))
                    i=i+1
            elif end1< list2[len(list2)-(result-len(list1))-2]: # 끝나는 지점에서 2가 1보다 위에 있는 경우
                while len(x1)-1 != result:
                    x1.append(min(end1, list2[len(list2)-(result-len(list1))-1+i]-1))
                    i=i+1

        elif add+len(list2)<len(list1): # 1이 2보다 늦게 끝나는 경우
            if end2> list1[add+len(list2)-1]: # 끝나는 지점에서 2가 1보다 위에 있는 경우
                while (add+len(x2)-1) != result:
                    x2.append(max(end2, list1[add+len(list2)+i]+1))
                    i=i+1
            elif end2< list1[add+len(list2)-1]: #끝나는 지점에서 1이 2보다 위에 있는 경우
                while (add+len(x2)-1) != result:
                    x2.append(min(end2, list1[add+len(list2)+i]-1))
                    i=i+1
        
        # crane2 의 전 경로 찍는 경우(추가)
        if add>0 : # add가 1이상일때만 crane2의 전경로가 필요함
            fe = list3[-1] # 앞클러스터의 끝점1
            se = list4[-1] # 앞클러스터의 끝점2
            fs = list1[0] # 뒷클러스터의 시작점(우선순위1)
            ss = list2[0] # 뒷클러스터의 시작점(우선순위2) -> 얘가핵심임..
            a = abs(fs-fe) + abs(ss-se)
            b = abs(fs-se) + abs(ss-fe)
            maxa = max(abs(fs-fe),abs(ss-se))
            maxb = max(abs(fs-se),abs(ss-fe))

            if a > b and maxb > tran:    #ss는 fe와 match    
                if  fe > ss:
                    for i in range(1,add+1):
                        x2.insert(0,ss+i)
                elif fe < ss:  
                    for i in range(1,add+1):
                        x2.insert(0,ss-i)
            elif a < b and maxa > tran: #ss는 se와 match    
                if  se > ss:
                    for i in range(1,add+1):
                        x2.insert(0,ss+i)
                elif se < ss:  
                    for i in range(1,add+1):
                        x2.insert(0,ss-i)
            else:
                # 예외처리 end1 과 start2 가 동일한 경우
                if end1==start2 and add == len(list1):
                    if list1[add-3] >= list1[add-2]: #list1감소, 정지
                        for i in range(add):
                            x2.insert(0,end1-1)
                    elif list1[add-3] < list1[add-2]: # list1 상승
                        for i in range(add):
                            x2.insert(0,end1+1)

                elif add == len(list1): #예외처리(list1의 길이만큼밀때)- 2가 시작할 때 위치관계를 알 수 없음
                    if list1[add-3] > list1[add-2]: #list1감소
                        for i in range(add):
                            x2.insert(i,max(start2, list1[i]+1))
                    elif list1[add-3] < list1[add-2]: #list1증가
                        for i in range(add):
                            x2.insert(i,min(start2, list1[i]-1))
                else:
                    if start2 > list1[add]:
                        for i in range(add):
                            x2.insert(i,max(start2, list1[i]+1))
                    elif start2 < list1[add]:
                        for i in range(add):
                            x2.insert(i,min(start2, list1[i]-1))             

        return  (x1, x2)

    def plot_all(self):
        perm = list(permutations(self.order, 2))

        for i in range(len(perm)):
            self.cluster_set[(perm[i][0][2], perm[i][1][2])] = self.cluster(perm[i][0], perm[i][1])

        for i in perm:
            for k in perm:
                self.trans_dict[(k[0][2], k[1][2]), (i[0][2], i[1][2])] = self.get_trans(k[0][2], k[1][2], i[0][2], i[1][2],self.cluster_set)

        for i in perm:
            self.init_dict[(i[0][2], i[1][2])] = self.get_init(i[0][2], i[1][2], self.cluster_set)

        left = set(range(len(self.order)))
        temp_dict = {}
        min_times = []
        orders = []

        for i in perm: # perm은 order // i,k 순서에 대해 다시 생각해보자...
            for k in perm:
                temp_set=set()
                temp_set.add(i[0][2]) # 우선순위1 order index
                temp_set.add(i[1][2]) # 우선순위2 order index
                temp_set.add(k[0][2])
                temp_set.add(k[1][2])

                if len(temp_set)==4: #구성요소들간 같을때 조건문 추가?
                    self.trans_dict[(k[0][2],k[1][2]),(i[0][2],i[1][2])]=self.get_trans(k[0][2],k[1][2],i[0][2],i[1][2],self.cluster_set)

        for a in left:
            for b in left:
                if a!= b:
                    min_time, temp_order, temp_dict = self.repeat(a, b, left, temp_dict,self.init_dict,self.trans_dict,self.cluster_set)
                    min_times.append(min_time)
                    orders.append(temp_order)    

        minwls = min(min_times)
        min_index = min_times.index(minwls)
        min_order = list(reversed(orders[min_index]))

        t1=[]
        x1=[]
        t2=[]
        x2=[]
        temp_x1=[]
        temp_x2=[]

        for k in range(len(min_order)):
            if k == 0:
                temp=self.full_traj(self.cluster_set[min_order[k]][0], self.cluster_set[min_order[k]][1], self.cluster_set[min_order[k]][2],self.cluster_set[min_order[k]][3]) #cluster별 전체 traj 출력
                                #우선순위1인 order              #우선순위2인 order             # add                          #time
            else:
                temp=self.full_traj2(self.cluster_set[min_order[k-1]][0],self.cluster_set[min_order[k-1]][1],self.cluster_set[min_order[k]][0], self.cluster_set[min_order[k]][1], self.cluster_set[min_order[k]][2],self.cluster_set[min_order[k]][3],self.trans_dict[min_order[k-1],min_order[k]]) #cluster별 전체 traj 출력
            
            if temp[0][-1]>temp[1][-1]: #위에 있는 traj crane1으로 지정 (우선순위1끝점>우선순위2끝점)
                temp_x1.append(temp[0])
                temp_x2.append(temp[1])
            elif temp[0][-1]<temp[1][-1]: #(우선순위1끝점<우선순위2끝점)
                temp_x1.append(temp[1])
                temp_x2.append(temp[0])
        total_1=0
        for i in temp_x1:
            total_1=total_1+ len(i)
        total_2=0
        for i in temp_x2:
            total_2=total_2+ len(i)
        
        x1.append(20)
        x2.append(10)

        init_time=self.init_dict[min_order[0]]
        if abs(temp_x1[0][0]-x1[0])<init_time:
            for i in range(init_time-abs(temp_x1[0][0]-x1[0])):
                x1.append(x1[-1])
        if abs(temp_x2[0][0]-x2[0])<init_time:
            for i in range(init_time-abs(temp_x2[0][0]-x2[0])):
                x2.append(x2[-1])
                
        while temp_x1[0][0] > x1[-1]:
            x1.append(x1[-1]+1)
        while temp_x1[0][0] < x1[-1]:
            x1.append(x1[-1]-1)   
        while temp_x2[0][0] > x2[-1]:
            x2.append(x2[-1]+1)
        while temp_x2[0][0] < x2[-1]:
            x2.append(x2[-1]-1)        

        x1.pop()
        x2.pop()

        for k in temp_x1[0]:
            x1.append(k)
        for k in temp_x2[0]:
            x2.append(k)



        for k in range(1,len(temp_x1)):
            if self.trans_dict[min_order[k-1],min_order[k]]> abs(temp_x1[k][0] - x1[-1]): # 2의 transition time 이 더 길다. 
                for i in range(self.trans_dict[min_order[k-1],min_order[k]]- abs(temp_x1[k][0] - x1[-1])):
                    x1.append(x1[-1])
            if self.trans_dict[min_order[k-1],min_order[k]]> abs(temp_x2[k][0] - x2[-1]):
                for i in range(self.trans_dict[min_order[k-1],min_order[k]]- abs(temp_x2[k][0] - x2[-1])):
                    x2.append(x2[-1])
            trans1=0        
            while temp_x1[k][0] > x1[-1]:
                x1.append(x1[-1]+1)
                trans1=trans1+1
            while temp_x1[k][0] < x1[-1]:
                x1.append(x1[-1]-1)
                trans1=trans1+1
            x1.pop()   # 마지막에 좌표 한번 더 찍혀서 빼줌 

            trans2=0
            while temp_x2[k][0] > x2[-1]:
                x2.append(x2[-1]+1)
                trans2=trans2+1
            while temp_x2[k][0] < x2[-1]:
                x2.append(x2[-1]-1)
                trans2=trans2+1
            x2.pop()   # 마지막에 좌표 한번 더 찍혀서 빼줌 
            start1=0
            start2=0
            # if trans1>trans2:
            #     start1=abs(trans1-trans2)
            # if trans1<trans2:
            #     start2=abs(trans1-trans2)

            for q in range(start1, len(temp_x1[k])):
                x1.append(temp_x1[k][q])

            for q in range(start2,len(temp_x2[k])):       
                x2.append(temp_x2[k][q])
        if (len(x1) != len(x2)):
            print("%s 오류", k)
            print(len(x1),len(x2))
        
        t1=list(range(len(x1)))
        t2=list(range(len(x2)))
        plt.plot(t1,x1,c='r',label='crane1')
        plt.plot(t2,x2,c='b',label='crane2')
        plt.show()    

if __name__ == '__main__':
    crane_instance = Crane([[30,10,0],[1,6,1],[1,24,2],[30,24,3],[1,9,4],[19,30,5],[24,1,6],[3,30,7],[4,30,8],[4,1,9]])
    crane_instance.plot_all()