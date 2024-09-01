

for i = 1:1e6
    %%
    rng("shuffle");

    n = randi(intmax("int32"));
    n_collocation = 20;

    fn = sprintf("seed-%d-%d.csv", n, n_collocation);

    cmd_str = sprintf("parafoil.exe %d %d > %s", n, n_collocation, fn);

    disp(cmd_str);

    %%
    tic;
    system(cmd_str);
    toc
    %%

    % result = readmatrix(fn);
    % 
    % t = tiledlayout(4, 1);
    % 
    % nexttile([3, 1]);
    % plot(result(:,3), result(:, 4), 'LineWidth', 2)
    % xlabel('x(m)');
    % ylabel('y(m)');
    % axis equal;
    % grid on;
    % hold on
    % 
    % nexttile
    % plot(result(:,1), result(:, 6), 'LineWidth', 1);
    % xlabel('t(s)');
    % ylabel('u')
    % grid on
    % 
    % names = split(fn, ".");
    % 
    % savefig(names(1));

end