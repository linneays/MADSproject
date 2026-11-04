fc_delta = 0.015;
Tc = 0.03;
wc_delta = 2 * pi * fc_delta;
[Bh_delta, A_delta] = butter(3, wc_delta, 'high', 's');
Bl_delta = A_delta - Bh_delta;
Hhp_delta_s = tf(Bh_delta, A_delta);
Hlp_delta_s = tf(Bl_delta, A_delta);
Hhp_delta_z = c2d(Hhp_delta_s, Tc, 'tustin');
Hlp_delta_z = c2d(Hlp_delta_s, Tc, 'tustin');
[bh_delta, ah_delta] = tfdata(Hhp_delta_z, 'v');
[bl_delta, al_delta] = tfdata(Hlp_delta_z, 'v');


% ---- Kontroll: b_hp + b_lp = a (skal være ~1e-15)
% disp(max(abs(bh_delta + bl_delta - ah_delta)))

% ---- Bodeplott
f = logspace(-2, log10(0.99/(2*Tc)), 500);   % Hz, opp til like under Nyquist
w = 2*pi*f(:);                               % rad/s

G = @(sys) squeeze(freqresp(sys, w));
Hhp_s = G(Hhp_delta_s);   Hlp_s = G(Hlp_delta_s);
Hhp_z = G(Hhp_delta_z);   Hlp_z = G(Hlp_delta_z);
Hsum_z = Hhp_z + Hlp_z;                      % skal være 1 (0 dB, 0 grader)

figure
subplot(2,1,1)
semilogx(f, 20*log10(abs(Hhp_s)), 'b--', f, 20*log10(abs(Hlp_s)), 'r--'); hold on
semilogx(f, 20*log10(abs(Hhp_z)), 'b',   f, 20*log10(abs(Hlp_z)), 'r', ...
         f, 20*log10(abs(Hsum_z)), 'k', 'LineWidth', 1.5)
xline(fc_delta, ':', 'f_c')
grid on, ylim([-80 10])
ylabel('Amplitude [dB]')
title(sprintf('Komplementærfilter, f_c = %g Hz, T_c = %g s', fc_delta, Tc))
legend('HP (s)', 'LP (s)', 'HP (z)', 'LP (z)', 'HP+LP (z)', 'Location', 'southwest')

subplot(2,1,2)
semilogx(f, rad2deg(unwrap(angle(Hhp_z))), 'b', ...
         f, rad2deg(unwrap(angle(Hlp_z))), 'r', ...
         f, rad2deg(unwrap(angle(Hsum_z))), 'k', 'LineWidth', 1.5)
xline(fc_delta, ':', 'f_c')
grid on
xlabel('Frekvens [Hz]'), ylabel('Fase [grader]')