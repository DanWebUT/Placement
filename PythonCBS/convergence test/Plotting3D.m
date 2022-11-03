data = readtable("Sensitivity Analysis Percents Final Scrubbed.csv","NumHeaderLines",1);

data = renamevars(data,["Var1","Var2","Var3","Var4","Var5","Var6"],["Chance Mutation", "Chance Crossover", "Percent Elite", "Percent New", "Number of Generations", "Best Value"]);

figure
scatter3(data,'Percent Elite','Percent New','Number of Generations','filled','ColorVariable','Number of Generations')
colorbar