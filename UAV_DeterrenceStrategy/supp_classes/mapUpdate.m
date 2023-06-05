function data = mapUpdate(data, tau, nom)

    data = data*tau+(1-tau)*nom;
end