function err  = Kerror(A,K,M)
	a_int = trapz(A(1,:),A(2,:));
	k_int = trapz(K(1,:),K(2,:));
	m_int = trapz(M(1,:),M(2,:));

	err = [(k_int-a_int)/200;(m_int-a_int)/200];
end
