import heapq
import networkx as nx
import matplotlib.pyplot as plt
import pylab

G = []
terminals=[] 
Nodes=[]
g=[]
term=[]
alldist=[]
path=[]
cost=0
channa=nx.Graph()
arjun=nx.Graph()
arun=nx.Graph()

"""class Node:

    def __init__(self, value):
        super(Node, self).__init__()
        self.value = value
        self.rank=0
        self.path=[]
        self.dis=999


class DisjointSets:
    def makeset(self,i):
        return Node(i)
    def findset(self,i):
        if(i!=i.parent):
            p=findset(i.parent)
            return p
        else:
            return i
    def union(self,x,y):
        rx=self.findset(x)
        ry=self.findset(y)
        if(rx.rank>ry.rank):
            ry.parent=rx
        else:
            rx.parent=ry
            if(rx.rank==ry.rank):
                ry.rank+=1 """

def findpath(parent,i):
    global path
    if(parent[i]!=i):
        findpath(parent,parent[i])
       
    path.append(i)

def Dijistras(u):
    global G,path
    dis=[]
    d=[]
    parent=[]
    final=[]

    for i in range(len(G)):
        dis.append([999,i])
        d.append(999)
        parent.append(u)
    dis[u]=[0,u]
    d[u]=0
    parent[u]=u
    heapq.heapify(dis)
    while(len(dis)!=0):
        x=heapq.heappop(dis)
        s=x[1]
        for i in range(len(G[s])):
            f=G[s][i][0]
            
            for j in range(len(dis)):
                if(dis[j][1]==f):
                    if(d[s]+G[s][i][1]<d[f]):
                        d[f]=d[s]+G[s][i][1]
                        dis[j][0]=d[s]+G[s][i][1]
                        parent[f]=s
                        heapq.heapify(dis)
    for i in range(len(G)):
        findpath(parent,i)
        final.append([d[i],path])
        path=[]

    return final






    
        

"""def heapify(i):
    global g
    if((2*i+2)<len(g)):
        heapify(2*i+1)
        heapify(2*i+2)
        x=g[2*i+2]
        y=g[2*i+1]
        j=g[i]
        if(x[0]<y[0]):
            if(x[0]<j[0]):
                swap=g[2*i+2]
                g[2*i+2]=g[i]
                g[i]=swap
        elif(y[0]<j[0]):
            swap=g[2*i+1]
            g[2*i+1]=g[i]
            g[i]=swap
    elif((2*i+1)<len(g)):
        heapify(2*i+1)
        x=g[2*i+1]
        if(x[0]<i[0]):
            swap=g[2*i+1]
            g[2*i+1]=g[i]
            g[i]=swap"""

def stiner():
    global Nodes,G,terminals,g,alldist,term,cost,edges,channa,arjun,arun
    mallu=nx.Graph()
    shivu=nx.Graph()

    T=[]
    edges=[]
    
    small=[[999,[]],0,0]
    s=terminals[0]
    T.append(s)
    shivu.add_node(s)
    term.remove(s)
    while(len(term)!=0):
        for i in range(len(T)):
            for j in range(len(term)):
                if(alldist[T[i]][term[j]][0]<small[0][0]):
                        small=[alldist[T[i]][term[j]],T[i],term[j]]
        dist=small[0]
        treevertex=small[1]
        terminal=small[2]
        print(terminal)
        term.remove(terminal)
        paths=dist[1]
        for i in range(1,len(paths)):
            T.append(paths[i])
            x=paths[i]
            if(paths[i]==0):
                x=12
            mallu.add_node(paths[i],pos=(x,x))
            shivu.add_node(paths[i])
            #print(T)
            edges.append(str(paths[i-1]) + " - " + str(paths[i]))
            
            #print(edges)
            for j in range(len(G[paths[i-1]])):
                if(paths[i]==G[paths[i-1]][j][0]):
                    cost=cost+G[paths[i-1]][j][1]
                    
                    mallu.add_edge(paths[i-1],paths[i],color='b',weight=G[paths[i-1]][j][1])
                    arjun.add_edge(paths[i-1],paths[i],color='b',weight=G[paths[i-1]][j][1])
                    #print(cost)
            
        small=[[999,[]],0,0]
        finaledges=edges
    """print("The vertices in STEINER TREE are  =  "+ str(T))
    print("The edges    in STEINER TREE are  =  "+ str(edges))
    print("the  cost                         =  "+ str(cost))"""
    print("The vertices in STEINER TREE are  =  "+ str(mallu.nodes()))
    print("The edges    in STEINER TREE are  =  "+ str(mallu.edges()))
    print("the  cost                         =  "+ str(cost))
    for i in range(len(terminals)):
        arjun.remove_node(terminals[i])
        arjun.add_node(terminals[i],color='r')

    pos = nx.circular_layout(channa)

    edges = channa.edges()
    nodes = mallu.nodes()
    
    colors = [channa[u][v]['color'] for u,v in edges]
    weights = [channa[u][v]['weight'] for u,v in edges]

    nx.draw(channa, pos ,edges=edges, edge_color=colors, width=weights,with_labels=True)
    nx.draw_networkx_nodes(channa,pos,
                       node_color='r',
                       node_size=500,
                   alpha=0.8)
    plt.show()
    
    pos = nx.circular_layout(channa)

    edges = channa.edges()
    nodes = mallu.nodes()
    
    colors = [channa[u][v]['color'] for u,v in edges]
    weights = [channa[u][v]['weight'] for u,v in edges]

    nx.draw(channa, pos ,edges=edges, edge_color=colors, width=weights,with_labels=True)
    nx.draw_networkx_nodes(arun,pos,
                       node_color='g',
                       node_size=500,
                   alpha=0.8)
    plt.show()

    
    pos = nx.circular_layout(mallu)

    edges = mallu.edges()
    nodes = mallu.nodes()
    
    colors = [mallu[u][v]['color'] for u,v in edges]
    weights = [mallu[u][v]['weight'] for u,v in edges]

    nx.draw(mallu, pos ,edges=edges, edge_color=colors, width=weights,with_labels=True)
    nx.draw_networkx_nodes(arun,pos,
                       node_color='g',
                       node_size=500,
                   alpha=0.8)
    plt.show()
    


    

    

    

    pos = nx.circular_layout(arjun)

    edges = arjun.edges()
    nodes = arjun.nodes()
    
    colors = [arjun[u][v]['color'] for u,v in edges]
    weights = [arjun[u][v]['weight'] for u,v in edges]
    nodelist=terminals
    
    nx.draw(arjun, pos,edges=edges, edge_color=colors, width=weights,with_labels=True)
    nx.draw_networkx_nodes(shivu,pos,
                       node_color='g',
                       node_size=500,
                   alpha=0.8)
    '''nx.draw_networkx_nodes(G,pos,
                       nodelist=[],
                       node_color='g',
                       node_size=500,
                   alpha=0.8)'''
    plt.show()
    




    

    
    

def main():
    ''' Adjacency List representation. G is a list of lists. '''
    
    global Nodes,G,terminals,g,alldist,term,channa
    i=0
    #ds=DisjointSets()
    file=open('input.txt','r')
    for line in file:
        line=line.strip()
        adjacentVertices = []
        first=True
        for edge in line.split(' '):
            if first:
                first=False
                continue
            node,weight = edge.split(',')
            adjacentVertices.append((int(node),int(weight)))
            if(i<int(node)):
                channa.add_edge(i,int(node),color='b',weight=int(weight))
                arjun.add_edge(i,int(node),color='r',weight=int(weight))
        G.append(adjacentVertices)
        channa.add_node(i,color='g')
        arjun.add_node(i,color='g')
        i=i+1

    file.close()

    print(G)
    print(arjun.edges())

    """for i in range(len(G)):
        k=G[i]
        for j in range(len(G[i])):
            if(i<k[j][0]):
                g.append([k[j][1],i,k[j][0]])
    #print(g)"""
    n=int(input("Enter the no of terminals (n< " + str(len(G)) + ")  =  "))
    for i in range(n):
        k=int(input())
        if(k<len(G)):
            terminals.append(k)
            arun.add_node(k,color='g')
            #arjun.remove_node(k)
            arjun.add_node(k,color='g')
            channa.add_node(k,color='g')

        else:
            print(str(k) + "this is not a vertex in a graph")
            terminals.append(int(input()))

    #print(terminals)
    term=terminals
    """for i in range(len(G)):
        Nodes.append(ds.makeset(i))

    #print(Nodes)
    g.sort()
    #print(g)"""


    for i in range(len(G)):
        
        #p=Dijistras(i)
        #print("for vertex " + str(i) + " = ")
        alldist.append(Dijistras(i))
    #print(alldist)
    for i in range(len(terminals)):
        pass
    
    stiner()


if __name__ == '__main__':
    main()