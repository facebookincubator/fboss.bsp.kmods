# SPDX-License-Identifier: GPL-2.0+
# Copyright (c) Meta Platforms, Inc. and affiliates.

# The rpmbuild macros hardcoded __strip to be /usr/bin/strip.
# Change to /bin/llvm-strip to avoid lto compilation failure.
%define with_clang %{?using_clang: %{using_clang}} %{?!using_clang: 0}
%if %{with_clang}
%define __strip /bin/llvm-strip
%endif

%define has_kver %{?rpm_kernel_version: 1} %{?!rpm_kernel_version: 0}
%if !%{has_kver}
%define rpm_kernel_version %(uname -r)
%endif

%{!?kernel_module_package:%define kernel_module_package %nil}
%{!?kernel_module_package_buildreqs:%define kernel_module_package_buildreqs %nil}

%define debug_package %{nil}
%define kmod_dir /lib/modules/%{rpm_kernel_version}/extra/fboss/
%define util_dir /usr/local/fboss_bsp/%{rpm_kernel_version}/

Name: fboss_bsp_kmods
Summary: FBOSS BSP (Board Support Package) Kernel Modules
Version: 2.4.0
Release: 1
Vendor: Meta
License: GPLv2
Group: System Environment/Kernel
Source: %{name}-%{version}.tar.gz
BuildRoot: %{_tmppath}/%{name}-%{version}-root
BuildRequires: %kernel_module_package_buildreqs rsync tar gcc make kernel-devel rpm-build

%description
The BSP (Board Support Package) of FBOSS Switches manufactured by JDMs.

The package contains the kernel drivers for the FBOSS IOB and DOM FPGAs
(Multi-Function Devices), CPLDs and various leaf devices (such as hardware
monitoring sensors, etc).

%prep
%setup -q -n %{name}-%{version}

%build
export BUILD_KERNEL=%{rpm_kernel_version}
export BSP_VERSION=%{version}
make -C kmods clean
%if %{with_clang}
make CC=clang LD=ld.lld NM=llvm-nm -C kmods
%else
make -C kmods
%endif

%install
install -d %{buildroot}%{kmod_dir}
install -t %{buildroot}%{kmod_dir} kmods/*.ko
install -t %{buildroot}%{kmod_dir} kmods/*/*.ko
install -d %{buildroot}%{util_dir}
install -t %{buildroot}%{util_dir} kmods/scripts/fbsp-remove.sh
install -t %{buildroot}%{util_dir} kmods/scripts/kmods.json

%clean
rm -rf %{buildroot}

%package -n %{name}-%{rpm_kernel_version}
Summary: FBOSS BSP (Board Support Package) Kernel Modules
Group: System Environment/Kernel
Provides: %{name}

%description -n %{name}-%{rpm_kernel_version}
The BSP (Board Support Package) of FBOSS Switches manufactured by JDMs.

The package contains the kernel drivers for the FBOSS IOB and DOM FPGAs
(Multi-Function Devices), CPLDs and various leaf devices (such as hardware
monitoring sensors, etc).

%files -n %{name}-%{rpm_kernel_version}
%defattr (-, root, root)
%{kmod_dir}
%{util_dir}

%post -n %{name}-%{rpm_kernel_version}
/sbin/depmod -a %{rpm_kernel_version}

%postun -n %{name}-%{rpm_kernel_version}
/sbin/depmod -a %{rpm_kernel_version}

%changelog
* Mon Sep 23 2024 Tao Ren <taoren@meta.com> - 2.4.0
- move kmod list to kmods/scripts/kmods.json
- include MODULE_VERSION to all the kmods
* Mon Aug 12 2024 Tao Ren <taoren@meta.com> - 2.3.0-1
- fix compilation errors in Linux 6.9
- include more drivers for Janga and Tahan DVT
- a few bug fixes
* Fri Jul 12 2024 Tao Ren <taoren@meta.com> - 2.0.0-1
- include more drivers and fix bugs for Minipack3 DVT
* Wed Mar 13 2024 Tao Ren <taoren@meta.com> - 1.5.0-1
- first release candidate
* Thu Mar 9 2023 Tao Ren <taoren@meta.com> - 0.1.0-1
- Initial release
