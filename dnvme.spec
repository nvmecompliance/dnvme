%define _distro %(uname -r)

Name:           dnvme
Version:        %{_major}.%{_minor}
Release:        1%{?dist}
Summary:        NVM Express hardware compliance test suite kernel driver
Group:          System Environment/Kernel
License:        Commercial
URL:            http://www.intel.com
Source0:        %{name}-%{version}.tar.gz
BuildRoot:      %{_tmppath}/%{name}-%{version}-%{release}-root-%(%{__id_u} -n)

%description
NVM Express hardware compliance test suite kernel driver.

%prep
%setup -q

%build
%configure
make %{?_smp_mflags}

%install
rm -rf $RPM_BUILD_ROOT
mkdir -p $RPM_BUILD_ROOT/etc/udev/rules.d
make install DESTDIR=$RPM_BUILD_ROOT

%clean
rm -rf $RPM_BUILD_ROOT

%files
%defattr(644,root,root,755)
/etc/udev/rules.d/*
/lib/modules/%{_distro}/*

%post
/sbin/depmod -a

%preun

%postun
/sbin/depmod -a

%changelog
