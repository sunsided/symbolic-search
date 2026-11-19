begin_version
3
end_version
begin_metric
0
end_metric
3
begin_variable
var0
-1
2
Atom case-at(home)
Atom case-at(office)
end_variable
begin_variable
var1
-1
3
Atom at(paper, home)
Atom at(paper, office)
<none of those>
end_variable
begin_variable
var2
-1
2
Atom in(paper)
NegatedAtom in(paper)
end_variable
1
begin_mutex_group
2
0 0
0 1
end_mutex_group
begin_state
0
0
1
end_state
begin_goal
1
1 1
end_goal
4
begin_operator
move home office
0
2
0 0 0 1
1 2 0 1 -1 1
1
end_operator
begin_operator
move office home
0
2
0 0 1 0
1 2 0 1 -1 0
1
end_operator
begin_operator
put-in paper home
1
0 0
2
0 1 0 2
0 2 -1 0
1
end_operator
begin_operator
put-in paper office
1
0 1
2
0 1 1 2
0 2 -1 0
1
end_operator
0
