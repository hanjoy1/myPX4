function test_tailsitter_mode_simulink()
%TEST_TAILSITTER_MODE_SIMULINK 检查尾座式四状态切换逻辑。
p = [-1.05; -0.26; 0.5; 10.0];

% 初始状态为多旋翼。
mode = tailsitter_mode_simulink( ...
    false, 0.0, 0.0, 0.01, p);
assert(mode == uint8(0));

% 请求固定翼后进入前转换。
mode = tailsitter_mode_simulink( ...
    true, 0.0, 0.0, 0.01, p);
assert(mode == uint8(1));

% 俯仰角达到门限但空速不足，继续保持前转换。
mode = tailsitter_mode_simulink( ...
    true, -1.1, 8.0, 0.01, p);
assert(mode == uint8(1));

% 俯仰角和空速同时满足，进入固定翼。
mode = tailsitter_mode_simulink( ...
    true, -1.1, 11.0, 0.01, p);
assert(mode == uint8(2));

% 取消固定翼请求后进入后转换。
mode = tailsitter_mode_simulink( ...
    false, -1.1, 11.0, 0.01, p);
assert(mode == uint8(3));

% 抬头达到后转换俯仰角门限后进入多旋翼。
mode = tailsitter_mode_simulink( ...
    false, -0.2, 5.0, 0.01, p);
assert(mode == uint8(0));

disp('test_tailsitter_mode_simulink: PASS');
end
